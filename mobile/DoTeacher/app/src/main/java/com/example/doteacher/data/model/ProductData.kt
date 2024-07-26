package com.example.doteacher.data.model

import android.os.Parcelable
import kotlinx.parcelize.Parcelize


@Parcelize
data class ProductData(
    val name: String,
    val imageResId: Int,
    val explain: String
) : Parcelable
