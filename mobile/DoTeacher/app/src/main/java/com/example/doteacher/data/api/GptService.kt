package com.example.doteacher.data.api

import com.example.doteacher.data.model.GptData
import com.example.doteacher.data.model.param.GptParam
import retrofit2.http.Body
import retrofit2.http.POST

interface GptService {


    @POST("chat/completions")
    suspend fun sendChat(
        @Body gptParam: GptParam
    ): GptData
}