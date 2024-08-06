package com.example.doteacher.data.api

import com.example.doteacher.data.model.PhotoData
import com.example.doteacher.data.model.ResponseData
import okhttp3.MultipartBody
import okhttp3.RequestBody
import retrofit2.http.GET
import retrofit2.http.Multipart
import retrofit2.http.POST
import retrofit2.http.Part
import retrofit2.http.Path
import retrofit2.http.Streaming

interface PhotoService {

    @Multipart
    @POST("photo")
    suspend fun uploadPhoto(
        @Part file: MultipartBody.Part,
        @Part("description") description: RequestBody,
        @Part("userId") userId: RequestBody
    ): ResponseData<PhotoData>

    @GET("photo/{photoId}")
    suspend fun getPhoto(
        @Path("photoId") photoId: Int
    ): ResponseData<PhotoData>

    @GET("photos/{userId}")
    suspend fun getUserPhotos(@Path("userId") userId: Int): ResponseData<List<PhotoData>>

    @GET("photos")
    suspend fun getAllPhotos(): ResponseData<List<PhotoData>>

    @GET("photo/download/{fileName}")
    @Streaming
    suspend fun downloadPhoto(
        @Path("fileName") fileName: String
    ): ResponseData<okhttp3.ResponseBody>
}