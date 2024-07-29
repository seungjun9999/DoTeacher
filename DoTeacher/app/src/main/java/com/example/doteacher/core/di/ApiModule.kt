package com.example.doteacher.core.di

import com.example.doteacher.data.api.GptService
import com.example.doteacher.data.api.UserService
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import retrofit2.Retrofit
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object ApiModule {

    @Singleton
    @Provides
    fun provideUserService(
        @NetworkModule.BaseRetrofit retrofit: Retrofit
    ): UserService = retrofit.create(UserService::class.java)

    @Singleton
    @Provides
    fun provideGptService(
        @NetworkModule.GptRetrofit retrofit : Retrofit
    ) : GptService = retrofit.create(GptService::class.java)

}