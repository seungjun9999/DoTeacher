package com.example.doteacher.data.source

import com.example.doteacher.data.model.ProductData
import com.example.doteacher.data.model.ResponseData

interface ProductDataSource {

    suspend fun getAllProducts() : ResponseData<List<ProductData>>

    suspend fun getProduct(productId :Int) : ResponseData<ProductData>

    suspend fun getRandomProducts(count : Int) : ResponseData<List<ProductData>>
}