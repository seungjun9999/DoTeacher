package com.example.doteacher.ui.login

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
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

    private val _userSignUpSuccess = MutableLiveData<Boolean>()
    val userSignUpSuccess: LiveData<Boolean> get() = _userSignUpSuccess

    fun setUserSignUpSuccess(value: Boolean){
        _userSignUpSuccess.value =value
    }

    fun signUp(userParam: UserParam) {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.signUp(userParam)
            }) {
                is ResultWrapper.Success -> {
                    Timber.d("signup param $userParam")
                    SingletonUtil.user = response.data.data
                    setUserSignUpSuccess(true)
                    Timber.d("signup suc ${response.data.data}")
                }

                is ResultWrapper.GenericError -> {
                    setUserSignUpSuccess(false)
                    Timber.d("signup error ${response.message}")
                }

                is ResultWrapper.NetworkError -> {
                    setUserSignUpSuccess(false)
                    Timber.d("signup network error")
                }
            }
        }

    }
}