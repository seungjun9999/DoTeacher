package com.example.doteacher.data.api

import com.example.doteacher.data.model.ResponseData
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.model.param.UserParam
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.PUT
import retrofit2.http.Path
import retrofit2.http.Query

interface UserService {
    @POST("user")
    suspend fun userSignUp(@Body userParam: UserParam): ResponseData<UserData>

    @GET("user")
    suspend fun getUserInfo(@Query("userId") userEmail: String): ResponseData<UserData>

    @GET("users")
    suspend fun getUsers(): ResponseData<List<UserData>>

    @PUT("user/{userId}/preferences")
    suspend fun updateUserPreferences(
        @Path("userId") userId: Int,
        @Body preferences: List<String>
    ): ResponseData<UserData>
}