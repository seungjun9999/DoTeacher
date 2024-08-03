package com.example.doteacher.ui.gallery.viewmodel

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.PhotoData
import com.example.doteacher.data.source.PhotoDataSource
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject


@HiltViewModel
class GalleryViewModel @Inject constructor(
    private val photoDataSource: PhotoDataSource
) : ViewModel() {

    private val _userPhotos = MutableLiveData<List<PhotoData>>()
    val userPhotos: LiveData<List<PhotoData>> get() = _userPhotos

    fun setPhotos(value: List<PhotoData>?) {
        value?.let {
            _userPhotos.value = it
            Timber.d("Photos set: ${it.size}")
        }
    }
    init {
        SingletonUtil.user?.let { getUserPhotos(it.id) }
    }


    fun getUserPhotos(userId: Int) {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                photoDataSource.getUserPhotos(userId)
            }) {
                is ResultWrapper.Success -> {
                    setPhotos(response.data.data)
                    Timber.d("사용자 사진 조회 성공 ${response.data.data}")
                }
                is ResultWrapper.GenericError -> {
                    Timber.d("사용자 사진 조회 에러 ${response.message}")
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("사용자 사진 조회 네트워크 에러")
                }
            }
        }
    }
}