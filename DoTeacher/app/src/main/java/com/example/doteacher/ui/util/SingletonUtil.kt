package com.example.doteacher.ui.util

import com.example.doteacher.data.model.UserData
import timber.log.Timber

object SingletonUtil {

    var user: UserData? = null
        set(value) {
            field = value
            Timber.d("SingletonUtil.user updated: $value")
        }

    val gptUrl = " https://api.openai.com/v1/"
    val baseUrl = "http://i11d102.p.ssafy.io:8081/"
//    val baseUrl = "http://172.31.141.166:8080/" // 에뮬레이터
//val baseUrl = "http://10.0.2.2:8080/"

//    val baseUrl = "http://192.168.137.11:8080/"
//        val baseUrl = "http://192.168.137.19:8080/" //
    val chatGptApi = "sk-proj-uOdkjPz1u5Jk70TkPFhET3BlbkFJ0WCxkYzClnKkZJZGRTQE"

}