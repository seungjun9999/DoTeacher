package com.example.doteacher.ui.util

import com.google.firebase.auth.ktx.auth
import com.google.firebase.ktx.Firebase
import timber.log.Timber

fun firebaseAuthCheck(
    complete: (String) -> Unit,
    cancel: () -> Unit
) {
    val auth = Firebase.auth.currentUser

    if (auth != null) {
        auth.getIdToken(true).addOnCompleteListener { task ->
            if (task.isSuccessful) {
                complete(auth.email.toString())
            } else {
                cancel()
                Timber.d("파이어베이스 유저 조회 실패")
            }
        }
    } else {
        cancel()
        Timber.d("파이어베이스 유저 조회 실패")
    }

}