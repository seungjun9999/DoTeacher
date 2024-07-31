package com.example.doteacher.ui.product

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.doteacher.data.model.PhotoData
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
class ProductViewModel @Inject constructor(
    private val productDataSource: ProductDataSource
) : ViewModel() {
    private val _allProducts = MutableLiveData<List<ProductData>>()
    val allProducts: LiveData<List<ProductData>> get() = _allProducts

    fun setProducts(value: List<ProductData>) {
        value?.let {
            _allProducts.value = it
        }
    }

    fun getAllProducts() {
        viewModelScope.launch {
            when (val response = safeApiCall(Dispatchers.IO) {
                productDataSource.getAllProducts ()

            }){
                is ResultWrapper.Success -> {
                setProducts(response.data.data)
            }
                is ResultWrapper.GenericError -> {
                    Timber.d("전체 작품 조회 에러 ${response.message}")
                }
                is ResultWrapper.NetworkError -> {
                    Timber.d("전체 작품 조회 네트워크 에러")
                }


            }
        }
    }


}