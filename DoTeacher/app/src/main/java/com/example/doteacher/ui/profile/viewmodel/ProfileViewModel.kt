package com.example.doteacher.ui.profile.viewmodel

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject


class Event<out T>(private val content: T) {
    var hasBeenHandled = false
        private set

    fun getContentIfNotHandled(): T? {
        return if (hasBeenHandled) {
            null
        } else {
            hasBeenHandled = true
            content
        }
    }

    fun peekContent(): T = content
}


@HiltViewModel
class ProfileViewModel @Inject constructor(
    private val userDataSource: UserDataSource
) : ViewModel() {

    private val _userData = MutableLiveData<UserData>()
    val userData: LiveData<UserData> get() = _userData

    private val _updateResult = MutableLiveData<Event<Boolean>>()
    val updateResult: LiveData<Event<Boolean>> get() = _updateResult

    init {
        _userData.value = SingletonUtil.user
    }

    fun updateProfileImage(userId: Int, imageUrl: String) {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.updateProfileImage(userId, imageUrl)
            }) {
                is ResultWrapper.Success -> {
                    response.data.data?.let { updatedUser ->
                        _userData.value = updatedUser
                        SingletonUtil.user = updatedUser
                        _updateResult.value = Event(true)
                    }
                    Timber.d("프로필 이미지 업데이트 성공")
                }
                is ResultWrapper.GenericError -> {
                    _updateResult.value = Event(false)
                    Timber.d("프로필 이미지 업데이트 에러: ${response.message}")
                }
                is ResultWrapper.NetworkError -> {
                    _updateResult.value = Event(false)
                    Timber.d("프로필 이미지 업데이트 네트워크 에러")
                }
            }
        }
    }
}