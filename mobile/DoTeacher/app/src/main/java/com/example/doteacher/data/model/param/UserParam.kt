package com.example.doteacher.data.model.param

data class UserParam(
    val userEmail: String,
    val userName: String,
    val userImage: String,
    val preferences: List<String>? = null,
    var token: String? = null,
    val userTuto: Boolean,
    val prefSelect: Boolean
)