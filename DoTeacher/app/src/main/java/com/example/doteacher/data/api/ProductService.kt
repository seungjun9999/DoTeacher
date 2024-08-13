package com.example.doteacher.data.api

import com.example.doteacher.data.model.ProductData
import com.example.doteacher.data.model.ResponseData
import retrofit2.http.GET
import retrofit2.http.PUT
import retrofit2.http.Path
import retrofit2.http.Query

interface ProductService {


    @GET("products")
    suspend fun getAllProducts() : ResponseData<List<ProductData>>

    @GET
    suspend fun getProduct(
        @Path("productId") productId : Int
    ) :ResponseData<ProductData>

    @GET("products/random")
    suspend fun getRandomProducts(@Query("count") count: Int): ResponseData<List<ProductData>>

}