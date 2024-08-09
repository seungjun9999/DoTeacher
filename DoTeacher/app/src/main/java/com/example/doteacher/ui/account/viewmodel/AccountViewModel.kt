package com.example.doteacher.ui.account

import android.content.Context
import android.net.Uri
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.ui.util.S3Uploader
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class AccountViewModel @Inject constructor(
    private val userDataSource: UserDataSource
) : ViewModel() {

    private val _registrationState = MutableStateFlow<RegistrationState>(RegistrationState.Idle)
    val registrationState: StateFlow<RegistrationState> = _registrationState

    private var email: String = ""
    private var password: String = ""
    private var imageUrl: String = ""
    private var nickname: String = ""

    fun setEmailAndPassword(email: String, password: String) {
        this.email = email
        this.password = password
    }

    fun setImageUrl(url: String) {
        this.imageUrl = url
    }

    fun setNickname(nickname: String) {
        this.nickname = nickname
    }

    fun uploadImageToS3(context: Context, imageUri: Uri) {
        viewModelScope.launch {
            _registrationState.value = RegistrationState.Loading
            try {
                val imageUrl = withContext(Dispatchers.IO) {
                    S3Uploader.uploadImage(context, imageUri)
                }
                setImageUrl(imageUrl)
                _registrationState.value = RegistrationState.ImageUploaded
            } catch (e: Exception) {
                Timber.d("Image upload failed")
                _registrationState.value = RegistrationState.Error("이미지 업로드에 실패했습니다. 다시 시도해주세요.")
            }
        }
    }

    fun register() {
        viewModelScope.launch {
            _registrationState.value = RegistrationState.Loading
            try {
                val userParam = UserParam(
                    userEmail = email,
                    userName = nickname,
                    password = password,
                    userImage = imageUrl,
                    userTuto = false,
                    prefSelect = false
                )
                when (val response = safeApiCall(Dispatchers.IO){
                    userDataSource.register(userParam)
                }){
                    is ResultWrapper.Success -> {
                        _registrationState.value = RegistrationState.Success
                        Timber.d("Registration success")
                    }
                    is ResultWrapper.GenericError -> {
                        _registrationState.value = RegistrationState.Error(response.message ?: "Unknown error")
                    }
                    is ResultWrapper.NetworkError -> {
                        _registrationState.value = RegistrationState.Error("네트워크 에러 ")
                    }
                }
            } catch (e: Exception) {
                Timber.d("Registration error: ${e.message}")
                _registrationState.value = RegistrationState.Error(e.message ?: "Registration failed")
            }
        }
    }
}

sealed class RegistrationState {
    object Idle : RegistrationState()
    object Loading : RegistrationState()
    object Success : RegistrationState()
    object ImageUploaded : RegistrationState()
    data class Error(val message: String) : RegistrationState()
}