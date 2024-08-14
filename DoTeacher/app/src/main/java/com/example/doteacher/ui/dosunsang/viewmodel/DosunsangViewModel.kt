package com.example.doteacher.ui.dosunsang.viewmodel

import android.content.Context
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import dagger.hilt.android.qualifiers.ApplicationContext
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class DosunsangViewModel @Inject constructor(
    private val userDataSource: UserDataSource,
    @ApplicationContext private val context: Context
) : ViewModel() {

    private val _robotName = MutableLiveData<String>()
    val robotName: LiveData<String> = _robotName

    private val _robotNumber = MutableLiveData<Int>()
    val robotNumber: LiveData<Int> = _robotNumber

    private val _connectionState = MutableLiveData<ConnectionState>()
    val connectionState: LiveData<ConnectionState> = _connectionState

    init {
        _connectionState.value = ConnectionState.LOADING
        loadSavedData()
    }

    fun setRobotInfo(name: String, number: Int) {
        _robotName.value = name
        _robotNumber.value = number
        saveData()
    }

    private fun saveData() {
        val sharedPref = context.getSharedPreferences("DosunsangPrefs", Context.MODE_PRIVATE)
        with(sharedPref.edit()) {
            putString("robotName", _robotName.value)
            putInt("robotNumber", _robotNumber.value ?: -1)
            putInt("connectionState", _connectionState.value?.ordinal ?: 0)
            apply()
        }
    }

    private fun loadSavedData() {
        val sharedPref = context.getSharedPreferences("DosunsangPrefs", Context.MODE_PRIVATE)
        _robotName.value = sharedPref.getString("robotName", null)
        _robotNumber.value = sharedPref.getInt("robotNumber", -1).takeIf { it != -1 }
        _connectionState.value = ConnectionState.values()[sharedPref.getInt("connectionState", 0)]
    }

    fun getUserRobotState(userId : Int){
        Timber.d("Fetching robot state for user ID: $userId")
        viewModelScope.launch {
            when(val response = safeApiCall(Dispatchers.IO){
                userDataSource.getUserRobotState(userId)
            }){
                is ResultWrapper.Success -> {
                    val state = response.data.data
                    Timber.d("Received robot state: $state")
                    if(state == 0){
                        _connectionState.value = ConnectionState.DISCONNECTED
                    }else{
                        _connectionState.value = ConnectionState.CONNECTED
                    }
                    Timber.d("Connection state updated to: ${_connectionState.value}")
                }
                is ResultWrapper.GenericError -> {
                    Timber.d("Error fetching robot state: ${response.message}")
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("Network error while fetching robot state")
                }
            }
        }
    }



    fun recommend(robotId: Int, userEmail: String) {
        _connectionState.value = ConnectionState.LOADING
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.recommend(robotId, userEmail)
            }) {
                is ResultWrapper.Success -> {
                    val msg = response.data.msg
                    Timber.d("Recommend success: $msg")
                    _connectionState.value = ConnectionState.CONNECTED
                    saveData()
                    SingletonUtil.user?.let { userDataSource.userDescription(it.id,1) }
                }
                is ResultWrapper.GenericError -> {
                    val msg = response.message
                    Timber.d("Recommend error: $msg")
                    _connectionState.value = ConnectionState.DISCONNECTED
                    saveData()
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("네트워크 에러")
                    _connectionState.value = ConnectionState.DISCONNECTED
                    saveData()
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