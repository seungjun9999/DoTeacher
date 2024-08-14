package com.example.doteacher.ui.dosunsang.viewmodel

import android.content.Context
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import okhttp3.*
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class DosunsangViewModel @Inject constructor(
    private val userDataSource: UserDataSource
) : ViewModel() {


    private val _connectionState = MutableLiveData<ConnectionState>()
    val connectionState: LiveData<ConnectionState> = _connectionState

    init {
        _connectionState.value = ConnectionState.DISCONNECTED
    }



    fun recommend(userParam: UserParam,robotId : Int) {
        _connectionState.value = ConnectionState.LOADING
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.recommend(userParam, robotId)
            }) {
                is ResultWrapper.Success -> {
                    val msg = response.data.msg
                    Timber.d("Recommend success: $msg")
                    _connectionState.value = ConnectionState.CONNECTED
                }
                is ResultWrapper.GenericError -> {
                    val msg = response.message
                    Timber.d("Recommend error: $msg")
                    _connectionState.value = ConnectionState.DISCONNECTED
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("네트워크 에러")
                    _connectionState.value = ConnectionState.DISCONNECTED
                }
            }
        }
    }

    fun photo() {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.takePhoto(1)
            }) {
                is ResultWrapper.Success -> {
                    Timber.d("사진 성공")
                }
                is ResultWrapper.GenericError -> {
                    Timber.d("${response.message} 사진에러")
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("네트워크 에러")
                }
            }
        }
    }

    fun gonext() {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.goNext(1)
            }) {
                is ResultWrapper.Success -> {
                    Timber.d("다음으로 성공")
                }
                is ResultWrapper.GenericError -> {
                    Timber.d("${response.message} 다음으로 에러")
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("네트워크 에러")
                }
            }
        }
    }

    enum class ConnectionState {
        DISCONNECTED,
        LOADING,
        CONNECTED
    }

}

