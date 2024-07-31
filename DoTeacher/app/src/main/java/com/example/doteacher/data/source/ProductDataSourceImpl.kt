package com.example.doteacher.data.source

import com.example.doteacher.data.api.ProductService
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.data.model.ResponseData
import javax.inject.Inject

class ProductDataSourceImpl @Inject constructor(
    private val productService: ProductService
) : ProductDataSource{
    override suspend fun getAllProducts(): ResponseData<List<ProductData>> {
        return productService.getAllProducts()
    }

    override suspend fun getProduct(productId: Int): ResponseData<ProductData> {
       return productService.getProduct(productId)
    }

    override suspend fun getRandomProducts(count: Int): ResponseData<List<ProductData>> {
        return productService.getRandomProducts(count)
    }

}