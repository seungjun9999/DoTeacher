package com.example.doteacher.ui.preference.viewmodel

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
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
class PreferenceViewModel @Inject constructor(
    private val userDataSource: UserDataSource
) : ViewModel() {

    private val _preferencesUpdated = MutableLiveData<Boolean>()
    val preferencesUpdated: LiveData<Boolean> = _preferencesUpdated

    private val _userPreferences = MutableLiveData<List<String>>()
    val userPreferences: LiveData<List<String>> = _userPreferences

    fun loadUserPreferences() {
        viewModelScope.launch {
            SingletonUtil.user?.userEmail?.let { userEmail ->
                when (val response = safeApiCall(Dispatchers.IO) {
                    userDataSource.getUserInfo(userEmail)
                }) {
                    is ResultWrapper.Success -> {
                        val userData = response.data.data
                        _userPreferences.value = userData.preferences ?: emptyList()
                        Timber.d("User preferences loaded: ${userData.preferences}")
                    }
                    is ResultWrapper.GenericError -> {
                        Timber.d("Failed to load user preferences: ${response.message}")
                        _userPreferences.value = emptyList()
                    }
                    is ResultWrapper.NetworkError -> {
                        Timber.d("Network error while loading user preferences")
                        _userPreferences.value = emptyList()
                    }
                }
            } ?: run {
                Timber.d("User email is null")
                _userPreferences.value = emptyList()
            }
        }
    }

    fun updateUserPreferences(preferences: List<String>) {
        viewModelScope.launch {
            SingletonUtil.user?.id?.let { userId ->
                when (val response = safeApiCall(Dispatchers.IO) {
                    userDataSource.updateUserPreferences(userId, preferences)
                }) {
                    is ResultWrapper.Success -> {
                        SingletonUtil.user = SingletonUtil.user?.copy(preferences = preferences)
                        _preferencesUpdated.value = true
                        Timber.d("Preferences updated successfully: ${SingletonUtil.user?.preferences}")
                    }
                    is ResultWrapper.GenericError -> {
                        _preferencesUpdated.value = false
                        Timber.d("Preference update error ${response.message}")
                    }
                    is ResultWrapper.NetworkError -> {
                        _preferencesUpdated.value = false
                        Timber.d("Preference update network error")
                    }
                }
            } ?: run {
                _preferencesUpdated.value = false
                Timber.d("User ID is null")
            }
        }
    }
}