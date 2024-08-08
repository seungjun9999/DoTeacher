package com.example.doteacher.ui.util

import android.content.Context
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.firstOrNull
import kotlinx.coroutines.flow.map
import javax.inject.Inject
import javax.inject.Singleton


@Singleton
class TokenManager @Inject constructor(private val context: Context) {

    private object PreferencesKeys {
        val AUTH_TOKEN = stringPreferencesKey("auth_token")
        val USER_EMAIL = stringPreferencesKey("user_email")
    }

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

    suspend fun getToken(): String? {
        return context.dataStore.data.map { preferences ->
            preferences[PreferencesKeys.AUTH_TOKEN]
        }.firstOrNull()
    }

    suspend fun getEmail(): String? {
        return context.dataStore.data.map { preferences ->
            preferences[PreferencesKeys.USER_EMAIL]
        }.firstOrNull()
    }

    suspend fun saveToken(token: String) {
        context.dataStore.edit { preferences ->
            preferences[PreferencesKeys.AUTH_TOKEN] = token
        }
    }

    suspend fun isLoggedIn(): Boolean {
        return getToken() != null
    }
}