package com.example.doteacher.ui.home.viewmodel

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.data.source.ProductDataSource
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject


@HiltViewModel
class HomeViewModel @Inject constructor(
    private val productDataSource : ProductDataSource
)  : ViewModel() {

    private val _randomProducts = MutableLiveData<List<ProductData>>()
    val randomProducts: LiveData<List<ProductData>> get() = _randomProducts

    fun setProducts(value : List<ProductData>?){
        value.let {
            _randomProducts.value=it
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

}