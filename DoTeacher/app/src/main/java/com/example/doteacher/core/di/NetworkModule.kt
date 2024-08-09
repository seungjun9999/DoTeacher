package com.example.doteacher.core.di

import com.example.doteacher.data.api.PhotoService
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.TokenManager
import com.google.gson.GsonBuilder
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import kotlinx.coroutines.runBlocking
import okhttp3.Interceptor
import okhttp3.OkHttpClient
import okhttp3.logging.HttpLoggingInterceptor
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import retrofit2.converter.scalars.ScalarsConverterFactory
import timber.log.Timber
import java.util.concurrent.TimeUnit
import javax.inject.Named
import javax.inject.Qualifier
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object NetworkModule {

    @Qualifier
    @Retention(AnnotationRetention.BINARY)
    annotation class BaseRetrofit

    @Qualifier
    @Retention(AnnotationRetention.BINARY)
    annotation class GptRetrofit

    @Singleton
    @Provides
    @Named("LoggingInterceptor")
    fun provideLoggingInterceptor(): Interceptor {
        return HttpLoggingInterceptor { message ->
            Timber.d(message)
        }.apply {
            level = HttpLoggingInterceptor.Level.HEADERS
        }
    }

    @Singleton
    @Provides
    @Named("GptAuthInterceptor")
    fun provideGptAuthInterceptor(): Interceptor =
        Interceptor { chain ->
            with(chain) {
                val newRequest = request().newBuilder()
                    .addHeader("Authorization", "Bearer ${SingletonUtil.chatGptApi}")
                    .addHeader("Content-Type", "application/json")
                    .build()

                proceed(newRequest)
            }
        }

    @Singleton
    @Provides
    @Named("JwtAuthInterceptor")
    fun provideJwtAuthInterceptor(tokenManager: TokenManager): Interceptor =
        Interceptor { chain ->
            val original = chain.request()
            val requestBuilder = original.newBuilder()

            val tokenRequiredEndpoints = listOf("/authenticate")

            if (tokenRequiredEndpoints.any { original.url.encodedPath.contains(it) }) {
                runBlocking {
                    tokenManager.getToken()?.let { token ->
                        requestBuilder.addHeader("Authorization", "Bearer $token")
                    }
                }
            }

            chain.proceed(requestBuilder.build())
        }

    @Singleton
    @Provides
    @Named("BaseOkHttpClient")
    fun provideBaseOkHttpClient(
        @Named("LoggingInterceptor") loggingInterceptor: Interceptor,
        @Named("JwtAuthInterceptor") jwtAuthInterceptor: Interceptor
    ): OkHttpClient {
        return OkHttpClient.Builder()
            .connectTimeout(120, TimeUnit.SECONDS)
            .readTimeout(120, TimeUnit.SECONDS)
            .writeTimeout(120, TimeUnit.SECONDS)
            .addInterceptor(loggingInterceptor)
            .addInterceptor(jwtAuthInterceptor)
            .build()
    }

    @Singleton
    @Provides
    @Named("GptOkHttpClient")
    fun provideGptOkHttpClient(
        @Named("LoggingInterceptor") loggingInterceptor: Interceptor,
        @Named("GptAuthInterceptor") gptAuthInterceptor: Interceptor
    ): OkHttpClient {
        return OkHttpClient.Builder()
            .connectTimeout(120, TimeUnit.SECONDS)
            .readTimeout(120, TimeUnit.SECONDS)
            .writeTimeout(120, TimeUnit.SECONDS)
            .addInterceptor(loggingInterceptor)
            .addInterceptor(gptAuthInterceptor)
            .build()
    }

    @Singleton
    @Provides
    @BaseRetrofit
    fun provideBaseRetrofit(@Named("BaseOkHttpClient") okHttpClient: OkHttpClient): Retrofit {
        return Retrofit.Builder()
            .addConverterFactory(ScalarsConverterFactory.create())
            .addConverterFactory(GsonConverterFactory.create(GsonBuilder().setLenient().create()))
            .baseUrl(SingletonUtil.baseUrl)
            .client(okHttpClient)
            .build()
    }

    @Singleton
    @Provides
    @GptRetrofit
    fun provideGptRetrofit(@Named("GptOkHttpClient") okHttpClient: OkHttpClient): Retrofit {
        return Retrofit.Builder()
            .addConverterFactory(ScalarsConverterFactory.create())
            .addConverterFactory(GsonConverterFactory.create(GsonBuilder().setLenient().create()))
            .baseUrl(SingletonUtil.gptUrl)
            .client(okHttpClient)
            .build()
    }

    @Singleton
    @Provides
    fun providePhotoService(@BaseRetrofit retrofit: Retrofit): PhotoService {
        return retrofit.create(PhotoService::class.java)
    }

}