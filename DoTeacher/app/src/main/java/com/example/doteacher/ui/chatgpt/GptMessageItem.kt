package com.example.doteacher.ui.chatgpt

import com.example.doteacher.data.model.GptData

data class GptMessageItem(
    val role : String,
    val content : String
){

    companion object{
        val DEFAULT = emptyList<GptMessageItem>()
    }
}


fun GptData.Choice.toGptMessageItem() : GptMessageItem{
    return GptMessageItem(
        role = this.message.role,
        content = this.message.content
    )
}