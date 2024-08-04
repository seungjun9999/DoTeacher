package com.example.doteacher.data.source

import com.example.doteacher.data.api.UserService
import com.example.doteacher.data.model.ResponseData
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.model.param.UserParam
import javax.inject.Inject

class UserDataSourceImpl @Inject constructor(
    private val userService: UserService
) : UserDataSource {

    override suspend fun signUp(userParam: UserParam): ResponseData<UserData> {
        return userService.userSignUp(userParam)
    }

    override suspend fun getUserInfo(userEmail: String): ResponseData<UserData> {
        return userService.getUserInfo(userEmail)
    }

    override suspend fun getUsers(): ResponseData<List<UserData>> {
        return userService.getUsers()
    }

    override suspend fun updateUserPreferences(userId: Int, preferences: List<String>): ResponseData<UserData> {
        return userService.updateUserPreferences(userId, preferences)
    }

    override suspend fun updateUserTuto(userId: Int, userTuto: Boolean): ResponseData<UserData> {
        return userService.updateUserTuto(userId, userTuto)
    }

    override suspend fun updateProfileImage(userId: Int, imageUrl: String): ResponseData<UserData> {
        return userService.updateProfileImage(userId, imageUrl)
    }
}