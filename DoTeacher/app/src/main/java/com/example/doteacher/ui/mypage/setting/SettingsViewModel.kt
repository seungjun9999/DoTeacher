package com.example.doteacher.ui.mypage.setting

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.databinding.PreferItemImageBinding
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.TokenManager
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import com.google.firebase.auth.FirebaseAuth
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class SettingsViewModel @Inject constructor(
    private val tokenManager: TokenManager,
    private val userDataSource: UserDataSource
) : ViewModel() {

    private val _withdrawState = MutableLiveData<WithdrawState>()
    val withdrawState: LiveData<WithdrawState> = _withdrawState

    fun logout() {
        viewModelScope.launch {
            tokenManager.clearAllData()
            Timber.d("logout token is ${tokenManager.tokenFlow}, ${tokenManager.emailFlow}")
            FirebaseAuth.getInstance().signOut()
        }
    }

    fun withdrawUser() {
        viewModelScope.launch {
            _withdrawState.value = WithdrawState.Loading
            val userId = SingletonUtil.user?.id ?: run {
                _withdrawState.value = WithdrawState.Error("사용자 정보를 찾을 수 없습니다")
                return@launch
            }
            when (val response = safeApiCall(Dispatchers.IO) { userDataSource.deleteUser(userId) }) {
                is ResultWrapper.Success -> {
                    tokenManager.clearAllData()
                    SingletonUtil.user = null
                    FirebaseAuth.getInstance().signOut()
                    _withdrawState.value = WithdrawState.Success
                }
                is ResultWrapper.GenericError -> _withdrawState.value = WithdrawState.Error(response.message ?: "회원 탈퇴 실패")
                is ResultWrapper.NetworkError -> _withdrawState.value = WithdrawState.Error("네트워크 오류")
            }
        }
    }

    sealed class WithdrawState {
        object Loading : WithdrawState()
        object Success : WithdrawState()
        data class Error(val message: String) : WithdrawState()
    }
}