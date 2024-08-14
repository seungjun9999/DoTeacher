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
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import timber.log.Timber
import javax.inject.Inject


@HiltViewModel
class DosunsangViewModel @Inject constructor(
    private val userDataSource: UserDataSource
) :ViewModel(){

    private val _connectionState = MutableLiveData<ConnectionState>()
    val connectionState: LiveData<ConnectionState> = _connectionState

    private val _serverMessage = MutableLiveData<String>()
    val serverMessage: LiveData<String> = _serverMessage

    private var webSocket: WebSocket? = null

    fun initConnection(context: Context) {
        val sharedPref = context.getSharedPreferences("DosunsangPrefs", Context.MODE_PRIVATE)
        val isConnected = sharedPref.getBoolean("isConnected", false)
        _connectionState.value = if (isConnected) ConnectionState.CONNECTED else ConnectionState.DISCONNECTED
    }

    fun toggleConnection(context: Context) {
        when (_connectionState.value) {
            ConnectionState.DISCONNECTED -> {
                _connectionState.value = ConnectionState.CONNECTING
                connectWebSocket()
            }
            ConnectionState.CONNECTED -> {
                // 이미 연결된 상태에서는 아무 동작도 하지 않음
            }
            else -> {
                // 연결 중이거나 다른 상태일 때는 아무 동작도 하지 않음
            }
        }
    }

    private fun connectWebSocket() {
        val client = OkHttpClient()
        val request = Request.Builder().url("ws://i11d102.p.ssafy.io:8081/ws").build()
        webSocket = client.newWebSocket(request, object : WebSocketListener() {
            override fun onMessage(webSocket: WebSocket, text: String) {
                _serverMessage.postValue(text)
                when (text) {
                    "ack start" -> _connectionState.postValue(ConnectionState.CONNECTED)
                    "ack end" -> _connectionState.postValue(ConnectionState.DISCONNECTED)
                }
            }
        })
    }

    fun saveConnectionState(context: Context, isConnected: Boolean) {
        val sharedPref = context.getSharedPreferences("DosunsangPrefs", Context.MODE_PRIVATE)
        with (sharedPref.edit()) {
            putBoolean("isConnected", isConnected)
            apply()
        }
    }


    fun recommend(userParam: UserParam){
        viewModelScope.launch {
            when(val response = safeApiCall(Dispatchers.IO){
                userDataSource.recommend(userParam,1)
            }){
                is ResultWrapper.Success -> {
                    val msg = response.data.msg
                }
                is ResultWrapper.GenericError -> {
                    val msg = response.message
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("네트워크 에러")
                }
            }
        }
    }
    
    fun photo(){
        viewModelScope.launch { 
            when(val response = safeApiCall(Dispatchers.IO){
                userDataSource.takePhoto(1)
            }){
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

    fun gonext(){
        viewModelScope.launch {
            when(val response = safeApiCall(Dispatchers.IO){
                userDataSource.goNext(1)
            }){
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

    override fun onCleared() {
        super.onCleared()
        webSocket?.close(1000, "ViewModel is being cleared")
    }

}

enum class ConnectionState {
    DISCONNECTED, CONNECTING, CONNECTED
}