package com.example.doteacher.data.source

import com.example.doteacher.data.model.ResponseData
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.model.param.UserParam

interface UserDataSource {

    //회원가입
    suspend fun signUp(userParam: UserParam): ResponseData<UserData>

    //유저정보
    suspend fun getUserInfo(userEmail : String) : ResponseData<UserData>

    //유저 전체 정보
    suspend fun getUsers() : ResponseData<List<UserData>>

    //유저 취향 업데이트
    suspend fun updateUserPreferences(userId: Int, preferences: List<String>): ResponseData<UserData>

    suspend fun getUserByIdToken(idToken : String) : ResponseData<UserData>
}