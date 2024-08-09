package com.example.doteacher.ui.login

import android.animation.ObjectAnimator
import android.annotation.SuppressLint
import android.content.Intent
import android.view.View
import android.view.animation.LinearInterpolator
import android.widget.Toast
import androidx.activity.viewModels
import androidx.credentials.CredentialManager
import androidx.credentials.GetCredentialRequest
import androidx.credentials.exceptions.GetCredentialException
import androidx.lifecycle.lifecycleScope
import com.example.doteacher.R
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.databinding.ActivityLoginBinding
import com.example.doteacher.ui.account.AccountActivity
import com.example.doteacher.ui.account.AccountFragment
import com.example.doteacher.ui.base.BaseActivity
import com.example.doteacher.ui.main.MainActivity
import com.example.doteacher.ui.preference.PreferenceActivity
import com.example.doteacher.ui.util.SingletonUtil
import com.google.android.libraries.identity.googleid.GetGoogleIdOption
import com.google.android.libraries.identity.googleid.GoogleIdTokenCredential
import com.google.android.libraries.identity.googleid.GoogleIdTokenParsingException
import com.google.firebase.auth.GoogleAuthProvider
import com.google.firebase.auth.ktx.auth
import com.google.firebase.ktx.Firebase
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.launch
import timber.log.Timber
import java.security.MessageDigest
import java.util.UUID

@AndroidEntryPoint
class LoginActivity : BaseActivity<ActivityLoginBinding>(R.layout.activity_login) {

    private val loginViewModel: LoginViewModel by viewModels()

    private lateinit var credentialManager: CredentialManager
    private var rotateAnimation: ObjectAnimator? = null

    override fun init() {
        setupCredentialManager()
        setupClickListeners()
        setupLoadingAnimation()
        observeLoginState()
    }

    private fun setupCredentialManager() {
        credentialManager = CredentialManager.create(this)
    }

    private fun setupClickListeners() {
        binding.googlelogin.setOnClickListener { initiateGoogleLogin() }
        binding.guestlogin.setOnClickListener { initiateEmailLogin() }
        binding.tvSignup.setOnClickListener { navigateToSignUp() }
    }

    private fun observeLoginState() {
        lifecycleScope.launch {
            loginViewModel.loginState.collect { state ->
                when (state) {
                    is LoginState.Loading -> showLoading()
                    is LoginState.Success -> handleLoginSuccess()
                    is LoginState.Error -> handleLoginError(state.message)
                    LoginState.Idle -> hideLoading()
                    null -> {}
                }
            }
        }
    }

    @SuppressLint("Recycle")
    private fun setupLoadingAnimation() {
        if (rotateAnimation == null) {
            rotateAnimation = ObjectAnimator.ofFloat(binding.loadingBird, View.ROTATION, 0f, 360f).apply {
                duration = 2000
                repeatCount = ObjectAnimator.INFINITE
                interpolator = LinearInterpolator()
            }
        }
    }

    private fun showLoading() {
        binding.loadingVisible = true
        setupLoadingAnimation()
        rotateAnimation?.start()
    }

    private fun hideLoading() {
        binding.loadingVisible = false
        rotateAnimation?.cancel()
    }

    private fun handleLoginSuccess() {
        hideLoading()
        val intent = when {
            SingletonUtil.user?.prefSelect == true -> Intent(this, MainActivity::class.java)
            else -> Intent(this, PreferenceActivity::class.java)
        }
        startActivity(intent)

        if (SingletonUtil.user?.prefSelect == true) {
            finish()
        }

        finish()
    }

    private fun handleLoginError(message: String) {
        hideLoading()
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }

    private fun navigateToSignUp() {
        val intent = Intent(this, AccountActivity::class.java)
        startActivity(intent)
    }

    private fun initiateEmailLogin() {
        val email = binding.editTextText.text.toString()
        val password = binding.editTextText2.text.toString()
        if (email.isNotEmpty() && password.isNotEmpty()) {
            loginViewModel.login(email, password)
        } else {
            Toast.makeText(this, "이메일과 비밀번호를 입력해주세요.", Toast.LENGTH_SHORT).show()
        }
    }

    private fun initiateGoogleLogin() {
        Timber.d("Google login button clicked")
        val rawNonce = UUID.randomUUID().toString()
        val hashedNonce = hashNonce(rawNonce)

        val googleIdOption = GetGoogleIdOption.Builder()
            .setServerClientId(getString(R.string.web_application))
            .setFilterByAuthorizedAccounts(false)
            .setAutoSelectEnabled(true)
            .setNonce(hashedNonce)
            .build()

        val request = GetCredentialRequest.Builder()
            .addCredentialOption(googleIdOption)
            .build()

        lifecycleScope.launch {
            try {
                val result = credentialManager.getCredential(
                    request = request,
                    context = this@LoginActivity,
                )
                val credential = result.credential
                val googleIdTokenCredential = GoogleIdTokenCredential.createFrom(credential.data)
                handleGoogleCredential(googleIdTokenCredential)
            } catch (e: GetCredentialException) {
                handleLoginError(e.message ?: "Google Sign-In failed")
            } catch (e: GoogleIdTokenParsingException) {
                handleLoginError(e.message ?: "Failed to parse Google ID token")
            }
        }
    }

    private fun handleGoogleCredential(credential: GoogleIdTokenCredential) {
        val googleIdToken = credential.idToken
        if (googleIdToken != null) {
            val firebaseCredential = GoogleAuthProvider.getCredential(googleIdToken, null)
            Firebase.auth.signInWithCredential(firebaseCredential)
                .addOnCompleteListener(this) { task ->
                    if (task.isSuccessful) {
                        val user = Firebase.auth.currentUser
                        user?.let {
                            loginViewModel.signUp(
                                UserParam(
                                    userEmail = it.email ?: "",
                                    userName = it.displayName ?: "",
                                    password = "",
                                    userImage = it.photoUrl?.toString() ?: "",
                                    token = googleIdToken,
                                    userTuto = false,
                                    prefSelect = false
                                )
                            )
                        }
                    } else {
                        handleLoginError("Google login failed: ${task.exception?.message}")
                    }
                }
        } else {
            handleLoginError("GoogleIdToken is null")
        }
    }

    private fun hashNonce(rawNonce: String): String {
        val bytes = rawNonce.toByteArray()
        val md = MessageDigest.getInstance("SHA-256")
        val digest = md.digest(bytes)
        return digest.fold("") { str, it -> str + "%02x".format(it) }
    }
}