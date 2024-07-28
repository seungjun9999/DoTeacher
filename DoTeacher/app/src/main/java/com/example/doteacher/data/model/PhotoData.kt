package com.example.doteacher.data.model

import android.os.Parcelable
import kotlinx.parcelize.Parcelize

@Parcelize
data class PhotoData(
    val photoId: Int,
    val fileName: String,
    val description: String,
    val imageUrl: String,
    val userId: Int,
    val createdAt: String
) : Parcelable