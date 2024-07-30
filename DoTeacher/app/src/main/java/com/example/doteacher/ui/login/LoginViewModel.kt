package com.example.doteacher.ui.login

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject


@HiltViewModel
class LoginViewModel @Inject constructor(
    private val userDataSource: UserDataSource
) : ViewModel() {

    private val _navigateToMain = MutableLiveData<Boolean>()
    val navigateToMain: LiveData<Boolean> = _navigateToMain

    private val _userSignUpSuccess = MutableLiveData<UserData?>()
    val userSignUpSuccess: LiveData<UserData?> = _userSignUpSuccess

    private val _userHasPreferences = MutableLiveData<Boolean>()
    val userHasPreferences: LiveData<Boolean> = _userHasPreferences

    fun signUp(userParam: UserParam) {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.signUp(userParam)
            }) {
                is ResultWrapper.Success -> {
                    Timber.d("signup success ${response.data.data}")
                    SingletonUtil.user  = response.data.data
                    _userSignUpSuccess.value = response.data.data
                }
                else -> {
                    _userSignUpSuccess.value = null
                    Timber.d("signup failed")
                }
            }
        }
    }

    fun checkUser(token: String) {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.getUserByIdToken(token)
            }) {
                is ResultWrapper.Success -> {
                    response.data.data?.let { user ->
                        SingletonUtil.user = user
                        _userSignUpSuccess.value = user
                        checkUserPreferences(user)
                    }
                }
                else -> {
                    _userSignUpSuccess.value = null
                    Timber.d("Failed to get user by token")
                }
            }
        }
    }

    fun checkUserPreferences(user: UserData) {
        val hasPreferences = !user.preferences.isNullOrEmpty()
        _userHasPreferences.value = hasPreferences
        _navigateToMain.value = hasPreferences
    }
}