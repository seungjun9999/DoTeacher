package com.example.doteacher.data.model

data class UserData(
    val id: Int,
    val userEmail: String,
    val userName: String,
    val password: String? =null,
    val userImage: String,
    var preferences: List<String>? = null,
    var token: String,
    val userTuto: Boolean,
    val prefSelect: Boolean,
    val userProduct : Int? =null
)