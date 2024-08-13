package com.example.doteacher.ui.login

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.param.AuthenticationRequest
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.TokenManager
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class LoginViewModel @Inject constructor(
    private val userDataSource: UserDataSource,
    private val tokenManager: TokenManager
) : ViewModel() {

    private val _loginState = MutableStateFlow<LoginState>(LoginState.Idle)
    val loginState: StateFlow<LoginState> = _loginState

    private var isManualLogin = false

    init {
        checkSavedCredentials()
    }

    private fun checkSavedCredentials() {
        viewModelScope.launch {
            combine(tokenManager.tokenFlow, tokenManager.emailFlow) { token, email ->
                Pair(token, email)
            }.collect { (token, email) ->
                if (!isManualLogin && !token.isNullOrEmpty() && !email.isNullOrEmpty()) {
                    Timber.d("token is $token, email $email")
                    attemptAutoLogin(token, email)
                } else if (!isManualLogin) {
                    _loginState.value = LoginState.Idle
                }
            }
        }
    }

    private suspend fun attemptAutoLogin(token: String, email: String) {
        if (isManualLogin) return

        _loginState.value = LoginState.Loading
        Timber.d("Attempting auto login with email: $email")
        when (val response = safeApiCall(Dispatchers.IO) {
            userDataSource.getUserInfo(email)
        }) {
            is ResultWrapper.Success -> {
                val userData = response.data.data
                if (userData?.token == token) {
                    SingletonUtil.user = userData
                    _loginState.value = LoginState.Success
                } else {
                    tokenManager.clearAllData()
                    _loginState.value = LoginState.Idle  // Error 대신 Idle 상태로 변경
                }
            }
            else -> {
                Timber.e("Auto login failed: $response")
                tokenManager.clearAllData()
                _loginState.value = LoginState.Idle  // Error 대신 Idle 상태로 변경
            }
        }
    }

    fun signUp(userParam: UserParam) {
        viewModelScope.launch {
            _loginState.value = LoginState.Loading
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.getUserInfo(userParam.userEmail)
            }) {
                is ResultWrapper.Success -> {
                    // 사용자가 이미 존재하면 로그인을 진행합니다.
                    val existingUser = response.data.data
                    if (existingUser != null) {
                        Timber.d("User already exists, proceeding with login")
                        googlelogin(userParam.userEmail)
                    } else {
                        performSignUp(userParam)
                    }
                }
                is ResultWrapper.GenericError -> {
                    Timber.d("User not found or error occurred, attempting signup")
                    performSignUp(userParam)
                }
                is ResultWrapper.NetworkError -> {
                    _loginState.value = LoginState.Error("네트워크 오류")
                }
            }
        }
    }

    private suspend fun googlelogin(userEmail:String){
        when(val response = safeApiCall(Dispatchers.IO){
            userDataSource.getUserInfo(userEmail)
        }){
            is ResultWrapper.Success -> {
                val userData = response.data.data
                if(userData !=null){
                    SingletonUtil.user = userData
                    userData.token?.let { token ->
                        tokenManager.saveTokenAndEmail(token, userData.userEmail)
                    }
                    _loginState.value = LoginState.Success
                }
            }
            is  ResultWrapper.GenericError -> {
                _loginState.value = LoginState.Error("회원가입 실패 : 사용자 데이터 없음")
            }
            is ResultWrapper.NetworkError -> {
                _loginState.value = LoginState.Error("네트워크 오류")
            }
        }
    }

    private suspend fun performSignUp(userParam: UserParam) {
        when (val response = safeApiCall(Dispatchers.IO) {
            userDataSource.signUp(userParam)
        }) {
            is ResultWrapper.Success -> {
                val userData = response.data.data
                if (userData != null) {
                    SingletonUtil.user = userData
                    userData.token?.let { token ->
                        tokenManager.saveTokenAndEmail(token, userData.userEmail)
                    }
                    _loginState.value = LoginState.Success
                } else {
                    _loginState.value = LoginState.Error("회원가입 실패: 사용자 데이터 없음")
                }
            }
            is ResultWrapper.GenericError -> {
                _loginState.value = LoginState.Error("회원가입 오류: ${response.message}")
            }
            is ResultWrapper.NetworkError -> {
                _loginState.value = LoginState.Error("네트워크 오류")
            }
        }
    }

    fun login(email: String, password: String) {
        viewModelScope.launch {
            isManualLogin = true
            _loginState.value = LoginState.Loading

            when (val userInfoResponse = safeApiCall(Dispatchers.IO) {
                userDataSource.getUserInfo(email)
            }) {
                is ResultWrapper.Success -> {
                    val userData = userInfoResponse.data.data
                    if (userData != null) {
                        performLogin(email, password)
                    } else {
                        _loginState.value = LoginState.Error("사용자를 찾을 수 없습니다.")
                    }
                }
                is ResultWrapper.GenericError -> {
                    _loginState.value = LoginState.Error(userInfoResponse.message ?: "사용자 정보를 가져오는데 실패했습니다.")
                }
                is ResultWrapper.NetworkError -> {
                    _loginState.value = LoginState.Error("네트워크 오류: 사용자 정보를 가져오는데 실패했습니다.")
                }
            }

            isManualLogin = false
        }
    }

    private suspend fun performLogin(email: String, password: String) {
        when (val response = safeApiCall(Dispatchers.IO) {
            userDataSource.login(AuthenticationRequest(email, password))
        }) {
            is ResultWrapper.Success -> {
                val authResponse = response.data
                Timber.d("Auth response: $authResponse")
                if (authResponse.token.isNotBlank()) {
                    val tokenWithBearer = "Bearer ${authResponse.token}"
                    Timber.d("Saving token with Bearer: $tokenWithBearer")
                    tokenManager.saveTokenAndEmail(tokenWithBearer, email)
                    fetchUserInfo(email, authResponse.token)
                } else {
                    Timber.e("Invalid auth response: $authResponse")
                    _loginState.value = LoginState.Error("로그인 실패: 유효하지 않은 응답")
                }
            }
            is ResultWrapper.GenericError -> {
                Timber.e("Login error: ${response.message}")
                when (response.code) {
                    401 -> _loginState.value = LoginState.Error("비밀번호가 올바르지 않습니다.")
                    404 -> _loginState.value = LoginState.Error("등록되지 않은 이메일입니다.")
                    else -> _loginState.value = LoginState.Error("로그인 오류: ${response.message}")
                }
            }
            is ResultWrapper.NetworkError -> {
                Timber.e("Network error")
                _loginState.value = LoginState.Error("네트워크 오류")
            }
        }
    }

    private suspend fun fetchUserInfo(email: String, token: String) {
        when (val userInfoResponse = safeApiCall(Dispatchers.IO) {
            userDataSource.getUserInfo(email)
        }) {
            is ResultWrapper.Success -> {
                val userData = userInfoResponse.data.data
                if (userData != null) {
                    SingletonUtil.user = userData
                    SingletonUtil.user!!.token = token

                    when (val updateTokenResponse = safeApiCall(Dispatchers.IO) {
                        userDataSource.updateUserToken(userData.id, token)
                    }) {
                        is ResultWrapper.Success -> {
                            Timber.d("Token updated in database successfully")
                        }
                        else -> {
                            Timber.e("Failed to update token in database")
                        }
                    }
                    _loginState.value = LoginState.Success
                } else {
                    _loginState.value = LoginState.Error("사용자 정보를 가져오는데 실패했습니다.")
                }
            }
            is ResultWrapper.GenericError -> {
                _loginState.value = LoginState.Error(userInfoResponse.message ?: "사용자 정보를 가져오는데 실패했습니다.")
            }
            is ResultWrapper.NetworkError -> {
                _loginState.value = LoginState.Error("네트워크 오류: 사용자 정보를 가져오는데 실패했습니다.")
            }
        }
    }

}

sealed class LoginState {
    object Idle : LoginState()
    object Loading : LoginState()
    object Success : LoginState()
    data class Error(val message: String) : LoginState()
}