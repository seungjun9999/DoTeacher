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
}