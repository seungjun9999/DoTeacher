package com.example.doteacher.data.source

import com.example.doteacher.data.model.AuthenticationResponse
import com.example.doteacher.data.model.MessageResponse
import com.example.doteacher.data.model.ResponseData
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.model.param.AuthenticationRequest
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.ui.util.server.ResultWrapper
import timber.log.Timber

interface UserDataSource {

    //회원가입
    suspend fun signUp(userParam: UserParam): ResponseData<UserData>

    //유저정보
    suspend fun getUserInfo(userEmail : String) : ResponseData<UserData>

    //유저 전체 정보
    suspend fun getUsers() : ResponseData<List<UserData>>

    //유저 취향 업데이트
    suspend fun updateUserPreferences(userId: Int, preferences: List<String>): ResponseData<UserData>

    //유저 튜토 안내
    suspend fun updateUserTuto(userId: Int, userTuto: Boolean): ResponseData<UserData>

    // 프로필 이미지 업데이트
    suspend fun updateProfileImage(userId: Int, imageUrl: String): ResponseData<UserData>

    suspend fun login(authRequest: AuthenticationRequest): AuthenticationResponse

    suspend fun register(userParam: UserParam): ResponseData<MessageResponse>

    suspend fun deleteUser(userId: Int) : ResponseData<Unit>

    suspend fun updateUserToken(userId: Int, token: String): ResponseData<UserData>

    suspend fun recommend(userEmail: String, robotId : Int) : ResponseData<String>

    suspend fun userDescription(userId: Int, userDes: Int) : ResponseData<UserData>

    suspend fun takePhoto(robotId: Int) : ResponseData<MessageResponse>

    suspend fun goNext(robotId: Int) : ResponseData<MessageResponse>

    suspend fun updateUserName(userId: Int, userName : String) : ResponseData<UserData>
}