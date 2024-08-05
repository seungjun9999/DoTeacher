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
    val baseUrl = "http://i11d102.p.ssafy.io:8081/" // 핫스팟 로컬
//    val baseUrl = "http://10.0.2.2:8080/" // 에뮬레이터

    //    val baseUrl = "http://43.202.32.75:8081/" // docker
    val chatGptApi = "sk-proj-uOdkjPz1u5Jk70TkPFhET3BlbkFJ0WCxkYzClnKkZJZGRTQE"

}