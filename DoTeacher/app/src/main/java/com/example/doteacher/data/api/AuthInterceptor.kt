package com.example.doteacher.data.api

import com.example.doteacher.ui.util.TokenManager
import kotlinx.coroutines.runBlocking
import okhttp3.Interceptor
import okhttp3.Response
import javax.inject.Inject

class AuthInterceptor @Inject constructor(private val tokenManager: TokenManager) : Interceptor {
    override fun intercept(chain: Interceptor.Chain): Response {
        val request = chain.request().newBuilder()
        runBlocking {
            tokenManager.getToken()?.let { token ->
                request.addHeader("Authorization", "Bearer $token")
            }
        }
        return chain.proceed(request.build())
    }
}