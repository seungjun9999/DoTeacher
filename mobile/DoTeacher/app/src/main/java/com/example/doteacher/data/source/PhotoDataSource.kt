package com.example.doteacher.data.source

import com.example.doteacher.data.model.PhotoData
import com.example.doteacher.data.model.ResponseData
import okhttp3.MultipartBody
import okhttp3.RequestBody
import okhttp3.ResponseBody

interface PhotoDataSource {

    // 사진 업로드
    suspend fun uploadPhoto(file: MultipartBody.Part, description: RequestBody, userId: RequestBody): ResponseData<PhotoData>

    // 특정 사진 정보 가져오기
    suspend fun getPhoto(photoId: Int): ResponseData<PhotoData>

    // 모든 사진 정보 가져오기
    suspend fun getAllPhotos(): ResponseData<List<PhotoData>>

    // 사진 다운로드
    suspend fun downloadPhoto(fileName: String): ResponseData<ResponseBody>

    // 어떤 유저의 사진 정보 가져오기
    suspend fun getUserPhotos(userId: Int): ResponseData<List<PhotoData>>
}