package com.example.doteacher.ui.util

import android.content.Context
import android.graphics.Bitmap
import com.example.doteacher.BuildConfig
import com.amazonaws.auth.BasicAWSCredentials
import com.amazonaws.services.s3.AmazonS3Client
import com.amazonaws.services.s3.model.CannedAccessControlList
import com.amazonaws.services.s3.model.ObjectMetadata
import com.amazonaws.services.s3.model.PutObjectRequest
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.util.UUID

object S3Uploader {
    private val BUCKET_NAME = BuildConfig.AWS_BUCKET_NAME
    private val ACCESS_KEY = BuildConfig.AWS_ACCESS_KEY
    private val SECRET_KEY = BuildConfig.AWS_SECRET_KEY

    private val s3Client by lazy {
        val credentials = BasicAWSCredentials(ACCESS_KEY, SECRET_KEY)
        AmazonS3Client(credentials)
    }

    suspend fun uploadImage(context: Context, bitmap: Bitmap): String {
        val compressedImage = compressImage(bitmap)
        return uploadCompressedImage(compressedImage)
    }

    private fun compressImage(bitmap: Bitmap): ByteArray {
        val outputStream = ByteArrayOutputStream()
        bitmap.compress(Bitmap.CompressFormat.JPEG, 70, outputStream)
        return outputStream.toByteArray()
    }

    private fun uploadCompressedImage(imageBytes: ByteArray): String {
        val objectKey = "profile_images/${UUID.randomUUID()}.jpg"
        val inputStream = ByteArrayInputStream(imageBytes)
        val metadata = ObjectMetadata().apply {
            contentLength = imageBytes.size.toLong()
            contentType = "image/jpeg"
        }

        val putObjectRequest = PutObjectRequest(BUCKET_NAME, objectKey, inputStream, metadata)
            .withCannedAcl(CannedAccessControlList.PublicRead)

        s3Client.putObject(putObjectRequest)

        return s3Client.getUrl(BUCKET_NAME, objectKey).toString()
    }
}