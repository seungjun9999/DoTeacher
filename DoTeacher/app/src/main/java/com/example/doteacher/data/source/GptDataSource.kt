package com.example.doteacher.data.source

import com.example.doteacher.data.model.GptData
import com.example.doteacher.data.model.param.GptParam

interface GptDataSource {
    suspend fun sendChat(gptParam: GptParam) : GptData
}