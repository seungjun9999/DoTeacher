package com.example.doteacher.ui.util

import androidx.datastore.preferences.core.stringPreferencesKey

object PreferencesKeys {
    val AUTH_TOKEN = stringPreferencesKey("auth_token")
    val USER_EMAIL = stringPreferencesKey("user_email")
}