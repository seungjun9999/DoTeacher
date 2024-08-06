package com.example.doteacher.data.model.param

data class GptParam (
    val messages: List<Message>,
    val model: String
) {
    data class Message(
        val content: String,
        val role: String
    )
}
