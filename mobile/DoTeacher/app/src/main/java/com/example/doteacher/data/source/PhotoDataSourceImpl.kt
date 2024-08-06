package com.example.doteacher.data.source

import com.example.doteacher.data.api.PhotoService
import com.example.doteacher.data.model.PhotoData
import com.example.doteacher.data.model.ResponseData
import okhttp3.MultipartBody
import okhttp3.RequestBody
import okhttp3.ResponseBody
import javax.inject.Inject

class PhotoDataSourceImpl @Inject constructor(
    private val photoService: PhotoService
) : PhotoDataSource{
    override suspend fun uploadPhoto(
        file: MultipartBody.Part,
        description: RequestBody,
        userId: RequestBody
    ): ResponseData<PhotoData> {
        return photoService.uploadPhoto(file, description, userId)
    }

    override suspend fun getPhoto(photoId: Int): ResponseData<PhotoData> {
       return photoService.getPhoto(photoId)
    }

    override suspend fun getAllPhotos(): ResponseData<List<PhotoData>> {
        return photoService.getAllPhotos()
    }

    override suspend fun downloadPhoto(fileName: String): ResponseData<ResponseBody> {
        return photoService.downloadPhoto(fileName)
    }

    override suspend fun getUserPhotos(userId: Int): ResponseData<List<PhotoData>> {
        return photoService.getUserPhotos(userId)
    }

}