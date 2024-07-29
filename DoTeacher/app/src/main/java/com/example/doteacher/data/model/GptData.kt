package com.example.doteacher.data.model

import com.google.gson.annotations.SerializedName

data class GptData(
    val id: String,
    val `object`: String,
    val created: Long,
    val model: String,
    @SerializedName("system_fingerprint") val systemFingerprint: String,
    val choices: List<Choice>,
    val usage: Usage
) {

    data class Choice(
        val index: Int,
        val message: Message,
        val logprobs: Any?, // logprobs가 null이므로 Any?로 설정
        @SerializedName("finish_reason") val finishReason: String
    ) {

        data class Message(
            val role: String,
            val content: String
        )
    }
    data class Usage(
        @SerializedName("prompt_tokens")val promptTokens: Int,
        @SerializedName("completion_tokens")val completionTokens: Int,
        @SerializedName("total_tokens")val totalTokens: Int
    )
}