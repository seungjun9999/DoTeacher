package com.example.doteacher.ui.util

import android.content.Context
import androidx.datastore.preferences.core.edit
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import javax.inject.Inject
import javax.inject.Singleton


@Singleton
class TokenManager @Inject constructor(private val context: Context) {

    suspend fun saveTokenAndEmail(token: String, email: String) {
        context.dataStore.edit { preferences ->
            preferences[PreferencesKeys.AUTH_TOKEN] = token
            preferences[PreferencesKeys.USER_EMAIL] = email
        }
    }

    val tokenFlow: Flow<String?> = context.dataStore.data
        .map { preferences -> preferences[PreferencesKeys.AUTH_TOKEN] }

    val emailFlow: Flow<String?> = context.dataStore.data
        .map { preferences -> preferences[PreferencesKeys.USER_EMAIL] }

    suspend fun deleteTokenAndEmail() {
        context.dataStore.edit { preferences ->
            preferences.remove(PreferencesKeys.AUTH_TOKEN)
            preferences.remove(PreferencesKeys.USER_EMAIL)
        }
    }
}