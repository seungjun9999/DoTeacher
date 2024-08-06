package com.example.doteacher.ui.login

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
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
import java.sql.Time
import java.util.Timer
import javax.inject.Inject

@HiltViewModel
class LoginViewModel @Inject constructor(
    private val userDataSource: UserDataSource,
    private val tokenManager: TokenManager
) : ViewModel() {

    private val _loginState = MutableStateFlow<LoginState?>(null)
    val loginState: StateFlow<LoginState?> = _loginState

    init {
        checkSavedCredentials()
    }

    private fun checkSavedCredentials() {
        viewModelScope.launch {
            combine(tokenManager.tokenFlow, tokenManager.emailFlow) { token, email ->
                Pair(token, email)
            }.collect { (token, email) ->
                if (!token.isNullOrEmpty() && !email.isNullOrEmpty()) {
                    attemptAutoLogin(token, email)
                } else {
                    _loginState.value = LoginState.Idle
                }
            }
        }
    }

    private suspend fun attemptAutoLogin(token: String, email: String) {
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
                    // 토큰 불일치, 재로그인 필요
                    tokenManager.deleteTokenAndEmail()
                    _loginState.value = LoginState.Error("세션이 만료되었습니다. 다시 로그인해 주세요.")
                }
            }
            else -> {
                Timber.e("Auto login failed: $response")
                tokenManager.deleteTokenAndEmail()
                _loginState.value = LoginState.Error("자동 로그인 실패")
            }
        }
    }

    fun signUp(userParam: UserParam) {
        viewModelScope.launch {
            _loginState.value = LoginState.Loading
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.signUp(userParam)
            }) {
                is ResultWrapper.Success -> {
                    val userData = response.data.data
                    Timber.d("${response.data.data} user data")
                    if (userData != null) {
                        SingletonUtil.user = userData
                        userData.token?.let { token ->
                            tokenManager.saveTokenAndEmail(token, userData.userEmail)
                        }
                        _loginState.value = LoginState.Success
                    } else {
                        _loginState.value = LoginState.Error("로그인 실패: 사용자 데이터 없음")
                    }
                }
                is ResultWrapper.GenericError -> {
                    _loginState.value = LoginState.Error("로그인 오류: ${response.message}")
                }
                is ResultWrapper.NetworkError -> {
                    _loginState.value = LoginState.Error("네트워크 오류")
                }
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