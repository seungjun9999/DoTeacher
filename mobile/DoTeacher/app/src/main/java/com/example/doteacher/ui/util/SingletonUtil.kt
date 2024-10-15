package com.example.doteacher.ui.util

import com.example.doteacher.data.model.UserData

object SingletonUtil {

    var user : UserData?=null

//    val baseUrl = "http://192.168.254.166:8080/" // 핫스팟 로컬
    val baseUrl = "http://10.0.2.2:8080/" // 에뮬레이터

}