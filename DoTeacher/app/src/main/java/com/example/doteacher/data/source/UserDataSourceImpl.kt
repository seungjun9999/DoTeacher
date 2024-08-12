package com.example.doteacher.data.source

import com.example.doteacher.data.api.UserService
import com.example.doteacher.data.model.AuthenticationResponse
import com.example.doteacher.data.model.MessageResponse
import com.example.doteacher.data.model.ResponseData
import com.example.doteacher.data.model.UserData
import com.example.doteacher.data.model.param.AuthenticationRequest
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.ui.util.server.ResultWrapper
import com.example.doteacher.ui.util.server.safeApiCall
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

    override suspend fun login(authRequest: AuthenticationRequest): AuthenticationResponse {
        return userService.login(authRequest)
    }

    override suspend fun register(userParam: UserParam): ResponseData<MessageResponse> {
        return userService.register(userParam)
    }

    override suspend fun deleteUser(userId: Int): ResponseData<Unit> {
        return userService.deleteUser(userId)
    }

    override suspend fun updateUserToken(userId: Int, token: String): ResponseData<UserData> {
        return  userService.updateUserToken(userId, token)
    }

    override suspend fun recommend(userParam: UserParam, robotId: Int): ResponseData<UserData> {
        return userService.recommend(userParam,1)
    }


}