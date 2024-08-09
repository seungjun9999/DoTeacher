package com.example.doteacher.ui.home.viewmodel

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.source.ProductDataSource
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
class HomeViewModel @Inject constructor(
    private val productDataSource: ProductDataSource,
    private val userDataSource: UserDataSource
) : ViewModel() {

    private val _randomProducts = MutableLiveData<List<ProductData>>()
    val randomProducts: LiveData<List<ProductData>> get() = _randomProducts

    private val _userTutoUpdated = MutableLiveData<Boolean>()
    val userTutoUpdated: LiveData<Boolean> get() = _userTutoUpdated

    private val _userData = MutableLiveData<UserData>()
    val userData: LiveData<UserData> = _userData

    fun setProducts(value: List<ProductData>?) {
        value.let {
            _randomProducts.value = it
            if (it != null) {
                Timber.d("Products set : ${it.size}")
            }
        }
    }

    fun getRandomProducts() {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                productDataSource.getRandomProducts(10)  // count 파라미터 추가
            }) {
                is ResultWrapper.Success -> {
                    setProducts(response.data.data)
                    Timber.d("작품 조회 성공 ${response.data.data}")
                }
                is ResultWrapper.GenericError -> {
                    Timber.d("작품 조회 에러 ${response.message}")
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("작품 조회 네트워크 에러")
                }
            }
        }
    }

    fun updateUserTuto() {
        viewModelScope.launch {
            SingletonUtil.user?.id?.let { userId ->
                Timber.d("Updating UserTuto for user ID: $userId")
                when (val response = safeApiCall(Dispatchers.IO) {
                    userDataSource.updateUserTuto(userId, true)
                }) {
                    is ResultWrapper.Success -> {
                        val updatedUser = response.data.data
                        if (updatedUser != null) {
                            SingletonUtil.user = SingletonUtil.user?.copy(
                                userTuto = updatedUser.userTuto,
                                preferences = SingletonUtil.user?.preferences ?: updatedUser.preferences
                            )
                            _userTutoUpdated.value = true
                            Timber.d("UserTuto update success: ${SingletonUtil.user?.userTuto}, Preferences: ${SingletonUtil.user?.preferences}")
                        } else {
                            _userTutoUpdated.value = false
                            Timber.d("UserTuto update failed: user data is null")
                        }
                    }
                    is ResultWrapper.GenericError -> {
                        _userTutoUpdated.value = false
                        Timber.d("UserTuto update error: ${response.message}")
                    }
                    is ResultWrapper.NetworkError -> {
                        _userTutoUpdated.value = false
                        Timber.d("UserTuto update network error")
                    }
                }
            } ?: run {
                _userTutoUpdated.value = false
                Timber.d("UserTuto update failed: user ID is null")
            }
        }
    }

    fun loadUserData(email: String) {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                userDataSource.getUserInfo(email)
            }) {
                is ResultWrapper.Success -> {
                    val newUserData = response.data.data
                    _userData.postValue(newUserData)
                    SingletonUtil.user = SingletonUtil.user?.copy(
                        userTuto = newUserData.userTuto,
                        preferences = newUserData.preferences ?: emptyList()
                    ) ?: newUserData.copy(preferences = newUserData.preferences ?: emptyList())
                    Timber.d("User data reloaded in HomeViewModel: ${SingletonUtil.user}")
                }
                is ResultWrapper.GenericError -> {
                    Timber.e("Failed to reload user data in HomeViewModel: ${response.message}")
                }
                is ResultWrapper.NetworkError -> {
                    Timber.e("Network error while reloading user data in HomeViewModel")
                }
            }
        }
    }
}