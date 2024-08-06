package com.example.doteacher.data.source

import com.example.doteacher.data.api.GptService
import com.example.doteacher.data.model.GptData
import com.example.doteacher.data.model.param.GptParam
import javax.inject.Inject

class GptDataSourceImpl  @Inject constructor(
    private val gptService: GptService
) : GptDataSource{

    override suspend fun sendChat(gptParam: GptParam): GptData {
        return gptService.sendChat(gptParam)
    }

}