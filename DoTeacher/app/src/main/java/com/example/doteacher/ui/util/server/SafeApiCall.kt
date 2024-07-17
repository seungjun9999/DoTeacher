package com.example.doteacher.ui.util.server

import com.google.gson.Gson
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.withContext
import retrofit2.HttpException
import timber.log.Timber
import java.io.IOException

suspend fun <T> safeApiCall(
    dispatcher: CoroutineDispatcher,
    apiCall: suspend () -> T
): ResultWrapper<T> {
    return withContext(dispatcher) {
        try {
            ResultWrapper.Success(apiCall.invoke())
        } catch (throwable: Throwable) {
            when (throwable) {
                is IOException -> ResultWrapper.NetworkError
                is HttpException -> {
                    val code = throwable.code()
                    var message = ""
                    val errorBody = Gson().fromJson(
                        throwable.response()?.errorBody()?.string(),
                        ErrorBody::class.java
                    )
                    Timber.d("에러 바디 에러 $errorBody")
                    if (errorBody != null) {
//                        message = errorBody.data.message.ifEmpty { "" }

                    }
                    Timber.d("에러 바디 데이터 확인 $message")


                    ResultWrapper.GenericError(code, message)
                }

                else -> {

                    Timber.d("에러 바디 확인쓰 Generic ${throwable}")
                    ResultWrapper.GenericError(null, throwable.toString())
                }
            }
        }
    }

}