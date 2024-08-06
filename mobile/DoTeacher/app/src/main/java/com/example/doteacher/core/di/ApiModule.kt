package com.example.doteacher.core.di

import android.content.Context
import com.example.doteacher.data.api.GptService
import com.example.doteacher.data.api.ProductService
import com.example.doteacher.data.api.UserService
import com.example.doteacher.ui.util.TokenManager
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.android.qualifiers.ApplicationContext
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


    @Singleton
    @Provides
    fun provideProductService(
        @NetworkModule.BaseRetrofit retrofit: Retrofit
    ) : ProductService = retrofit.create(ProductService::class.java)

    @Singleton
    @Provides
    fun provideTokenManager(@ApplicationContext context: Context): TokenManager {
        return TokenManager(context)
    }

}