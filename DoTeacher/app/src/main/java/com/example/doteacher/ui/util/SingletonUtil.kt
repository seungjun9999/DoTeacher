package com.example.doteacher.ui.util

import com.example.doteacher.data.model.UserData

object SingletonUtil {

    var user : UserData?=null


    val gptUrl = " https://api.openai.com/v1/"
//    val baseUrl = "http://192.168.254.166:8080/" // 핫스팟 로컬
    val baseUrl = "http://10.0.2.2:8080/" // 에뮬레이터
    val chatGptApi = "sk-proj-uOdkjPz1u5Jk70TkPFhET3BlbkFJ0WCxkYzClnKkZJZGRTQE"
}