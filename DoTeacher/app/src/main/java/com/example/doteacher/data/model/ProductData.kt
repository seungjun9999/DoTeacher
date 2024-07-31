package com.example.doteacher.data.model

import android.os.Parcelable
import kotlinx.parcelize.Parcelize


@Parcelize
data class ProductData(
    val productId : Int,
    val productName: String,
    val productUrl: String,
    val productDesc: String
) : Parcelable
