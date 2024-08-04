package com.example.doteacher.ui.util

import android.content.Context
import android.net.Uri
import com.example.doteacher.BuildConfig
import com.amazonaws.auth.BasicAWSCredentials
import com.amazonaws.services.s3.AmazonS3Client
import com.amazonaws.services.s3.model.CannedAccessControlList
import com.amazonaws.services.s3.model.PutObjectRequest
import java.io.File
import java.util.UUID

object S3Uploader {
    private val BUCKET_NAME = BuildConfig.AWS_BUCKET_NAME
    private val ACCESS_KEY = BuildConfig.AWS_ACCESS_KEY
    private val SECRET_KEY = BuildConfig.AWS_SECRET_KEY

    private val s3Client by lazy {
        val credentials = BasicAWSCredentials(ACCESS_KEY, SECRET_KEY)
        AmazonS3Client(credentials)
    }

    suspend fun uploadImage(context: Context, imageUri: Uri): String {
        val file = File(context.cacheDir, "temp_image_${UUID.randomUUID()}")
        context.contentResolver.openInputStream(imageUri)?.use { input ->
            file.outputStream().use { output ->
                input.copyTo(output)
            }
        }

        val objectKey = "profile_images/${UUID.randomUUID()}"
        val putObjectRequest = PutObjectRequest(BUCKET_NAME, objectKey, file)
            .withCannedAcl(CannedAccessControlList.PublicRead)

        s3Client.putObject(putObjectRequest)

        file.delete() // 임시 파일 삭제

        return s3Client.getUrl(BUCKET_NAME, objectKey).toString()
    }
}