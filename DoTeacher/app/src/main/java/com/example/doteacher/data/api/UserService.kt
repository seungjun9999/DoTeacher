package com.example.doteacher.data.api

import com.example.doteacher.data.model.AuthenticationResponse
import com.example.doteacher.data.model.MessageResponse
import com.example.doteacher.data.model.ResponseData
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.model.param.AuthenticationRequest
import com.example.doteacher.data.model.param.UserParam
import retrofit2.http.Body
import retrofit2.http.DELETE
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.PUT
import retrofit2.http.Path
import retrofit2.http.Query

interface UserService {
    @POST("user")
    suspend fun userSignUp(@Body userParam: UserParam): ResponseData<UserData>

    @GET("user/{userEmail}")
    suspend fun getUserInfo(@Path("userEmail") userEmail: String): ResponseData<UserData>

    @GET("users")
    suspend fun getUsers(): ResponseData<List<UserData>>

    @PUT("user/{userId}/preferences")
    suspend fun updateUserPreferences(
        @Path("userId") userId: Int,
        @Body preferences: List<String>
    ): ResponseData<UserData>

    @PUT("user/{userId}/tuto")
    suspend fun updateUserTuto(@Path("userId") userId: Int, @Query("userTuto") userTuto: Boolean): ResponseData<UserData>

    @PUT("user/{userId}/name")
    suspend fun updateUserName(@Path("userId") userId: Int, @Query("username") userName:String) : ResponseData<UserData>

    @PUT("user/{userId}/profile-image")
    suspend fun updateProfileImage(
        @Path("userId") userId: Int,
        @Query("imageUrl") imageUrl: String
    ): ResponseData<UserData>

    @POST("authenticate")
    suspend fun login(@Body authRequest: AuthenticationRequest): AuthenticationResponse

    @POST("register")
    suspend fun register(@Body userParam: UserParam): ResponseData<MessageResponse>

    @DELETE("user/{userId}")
    suspend fun deleteUser(@Path ("userId") userId: Int) : ResponseData<Unit>

    @PUT("user/{userId}/prodDes")
    suspend fun userDescription(@Path("userId") userId: Int,@Query("userDes") userDes : Int) : ResponseData<UserData>

    @PUT("user/{userId}/token")
    suspend fun updateUserToken(@Path("userId") userId: Int, @Query("token") token: String): ResponseData<UserData>

    @POST("socket/recommend/{robotId}")
    suspend fun recommend(@Body userParam: UserParam, @Path("robotId") robotId: Int) : ResponseData<UserData>


    @POST("socket/picture/{robotId}")
    suspend fun takePhoto(@Path("robotId") robotId: Int) : ResponseData<MessageResponse>


    @POST("socket/next/{robotId}")
    suspend fun goNext( @Path("robotId") robotId: Int) : ResponseData<MessageResponse>
}