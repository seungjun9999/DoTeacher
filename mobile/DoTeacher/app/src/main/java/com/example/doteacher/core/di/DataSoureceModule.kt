package com.example.doteacher.core.di

import com.example.doteacher.data.source.GptDataSource
import com.example.doteacher.data.source.GptDataSourceImpl
import com.example.doteacher.data.source.PhotoDataSource
import com.example.doteacher.data.source.PhotoDataSourceImpl
import com.example.doteacher.data.source.ProductDataSource
import com.example.doteacher.data.source.ProductDataSourceImpl
import com.example.doteacher.data.source.UserDataSource
import com.example.doteacher.data.source.UserDataSourceImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton


@Module
@InstallIn(SingletonComponent::class)
interface DataSoureceModule {

    @Singleton
    @Binds
    fun provideUserDataSource(
        userDataSourceImpl: UserDataSourceImpl
    ): UserDataSource

    @Binds
    @Singleton
    fun providePhotoDataSource(
        photoDataSourceImpl: PhotoDataSourceImpl
    ): PhotoDataSource


    @Singleton
    @Binds
    fun provideGptDataSource(
        gptDataSourceImpl: GptDataSourceImpl
    ): GptDataSource


    @Singleton
    @Binds
    fun provideProductDataSource(
        productDataSourceImpl: ProductDataSourceImpl
    ) : ProductDataSource
}