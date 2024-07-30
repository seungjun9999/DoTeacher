package com.example.doteacher.ui.login

import android.animation.ObjectAnimator
import android.annotation.SuppressLint
import android.app.Activity
import android.content.Intent
import android.content.SharedPreferences
import android.view.View
import android.view.animation.LinearInterpolator
import android.widget.Toast
import androidx.activity.result.IntentSenderRequest
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import com.example.doteacher.R
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.databinding.ActivityLoginBinding
import com.example.doteacher.ui.base.BaseActivity
import com.example.doteacher.ui.main.MainActivity
import com.example.doteacher.ui.preference.PreferenceActivity
import com.example.doteacher.ui.util.initGoActivity
import com.google.android.gms.auth.api.identity.BeginSignInRequest
import com.google.android.gms.auth.api.identity.Identity
import com.google.android.gms.auth.api.identity.SignInClient
import com.google.android.gms.common.api.ApiException
import com.google.firebase.auth.GoogleAuthProvider
import com.google.firebase.auth.ktx.auth
import com.google.firebase.ktx.Firebase
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

val TAG: String = LoginActivity::class.java.getSimpleName()

@AndroidEntryPoint
class LoginActivity : BaseActivity<ActivityLoginBinding>(R.layout.activity_login) {


    private  val loginViewModel: LoginViewModel by viewModels()

    private lateinit var oneTapClient: SignInClient
    private lateinit var signInRequest: BeginSignInRequest
    private  var idToken: String? = null
    private lateinit var rotateAnimation: ObjectAnimator
    private lateinit var pref : SharedPreferences

    private val oneTapClientResult =
        registerForActivityResult(ActivityResultContracts.StartIntentSenderForResult()) { result ->
            if (result.resultCode == Activity.RESULT_OK) {
                try {
                    // 인텐트로 부터 로그인 자격 정보를 가져옴
                    val credential = oneTapClient.getSignInCredentialFromIntent(result.data)
                    // 가져온 자격 증명에서 Google ID 토큰을 추출
                    val googleIdToken = credential.googleIdToken
                    Timber.d("구글 아이디 토큰$googleIdToken")
                    if (googleIdToken != null) {
                        // Google ID 토큰을 사용해 Firebase 인증 자격 증명을 생성
                        val firebaseCredential = GoogleAuthProvider.getCredential(googleIdToken, null)
                        // 생성된 Firebase 인증 자격 증명을 사용하여 Firebase 에 로그인을 시도
                        Firebase.auth.signInWithCredential(firebaseCredential).addOnCompleteListener { task ->
                            if (task.isSuccessful) {
                                Firebase.auth.currentUser?.getIdToken(true)?.addOnCompleteListener { idTokenTask ->
                                    if (idTokenTask.isSuccessful) {
                                        idTokenTask.result?.token?.let { token ->
                                            Timber.d("email이메일 ${task.result.user?.email.toString()}")
                                            Timber.d("name이름 ${task.result.user?.displayName.toString()}")
                                            Timber.d("phot사진 ${task.result.user?.photoUrl}")
                                            idToken = token
                                            binding.loadingVisible = true
                                            loginViewModel.signUp(
                                                UserParam(
                                                    userEmail = task.result.user?.email.toString(),
                                                    userName = task.result.user?.displayName.toString(),
                                                    userImage = task.result.user?.photoUrl.toString(),
                                                    idToken = idToken!!
                                                )
                                            )
                                        } ?: Timber.e("FirebaseIdToken is null.")
                                    }
                                }
                            } else {
                                Timber.d("구글 로그인 오류 ${task.exception}")
                            }
                        }
                    } else {
                        Timber.e("GoogleIdToken is null.")
                    }
                } catch (exception: ApiException) {
                    Timber.d("구글 로그인 오류 ${exception.localizedMessage}")
                    Timber.e(exception.localizedMessage)
                }
            }
        }

    override fun init() {
        pref= getSharedPreferences("userToken", MODE_PRIVATE)
        setupLoadingAnimation()
        initGoogleLogin()
        checkAutoLogin()
        observeNavigation()
        clickGoogleLoginBtn()
        clickGuestLoginBtn()
        observeSignUpSuccess()
        observeUserPreferences()

    }

    private fun observeNavigation() {
        loginViewModel.navigateToMain.observe(this) { shouldNavigateToMain ->
            if (shouldNavigateToMain) {
                startActivity(Intent(this, MainActivity::class.java))
            } else {
                startActivity(Intent(this, PreferenceActivity::class.java))
            }
            finish()
        }
    }

    private fun checkAutoLogin(){
        pref.getString("userToken","")?.let { token ->
            if(token.isNotEmpty()){
                showLoading()
                loginViewModel.checkUser(token)
            }
        }
    }

    @SuppressLint("Recycle")
    private fun setupLoadingAnimation() {
        rotateAnimation = ObjectAnimator.ofFloat(binding.loadingBird, View.ROTATION, 0f, 360f).apply {
            duration = 2000
            repeatCount = ObjectAnimator.INFINITE
            interpolator = LinearInterpolator()
        }
    }

    private fun showLoading() {
        binding.loadingVisible = true
        rotateAnimation.start()
    }

    private fun hideLoading() {
        binding.loadingVisible = false
        rotateAnimation.cancel()

    }

    private fun observeSignUpSuccess() {
        loginViewModel.userSignUpSuccess.observe(this) { user ->
            if (user != null) {
                hideLoading()
                idToken?.let { token ->
                    pref.edit().putString("userToken", token).apply()
                }
            } else {
                hideLoading()
                Toast.makeText(this, "잠시후 다시 시도해주세요.", Toast.LENGTH_SHORT).show()
            }
            binding.loadingVisible = false
        }
    }

    private fun observeUserPreferences() {
        loginViewModel.userHasPreferences.observe(this) { hasPreferences ->
            if (hasPreferences) {
                startActivity(Intent(this, MainActivity::class.java))
            } else {
                startActivity(Intent(this, PreferenceActivity::class.java))
            }
            finish()
        }
    }



    private fun initGoogleLogin() {
        // 현쟈 액티비티(this)에 대한 Google One Tap 로그인 클라이언트를 가져옴
        oneTapClient = Identity.getSignInClient(this)
        signInRequest = BeginSignInRequest.builder()
            .setGoogleIdTokenRequestOptions(
                BeginSignInRequest.GoogleIdTokenRequestOptions.builder()
                    // Google Id Token 기반 로그인을 지원하도록 설정
                    .setSupported(true)
                    // 서버의 클라이언트 ID 를 설정
                    .setServerClientId(this.getString(R.string.web_application))
                    // 기존에 인증된 계정만을 필터링하지 않도록 설정
                    .setFilterByAuthorizedAccounts(false)
                    .build(),
            )
            // 이전에 선택 했던 계정을 기억
            .setAutoSelectEnabled(true)
            .build()
    }

    private fun clickGuestLoginBtn(){
        binding.guestlogin.setOnClickListener {  // 새로운 게스트 로그인 버튼 사용
                UserParam(
                    userEmail = "guest",
                    userName = "게스트",
                    userImage = null.toString(),
                    null,
                    ""
                )
            initGoActivity(this, MainActivity::class.java)
        }
    }


    private fun clickGoogleLoginBtn() {
        binding.googlelogin.setOnClickListener {
            Timber.d("Google login button clicked")
            showLoading()
            oneTapClient.beginSignIn(signInRequest)
                .addOnSuccessListener(this) { result ->
                    Timber.d("beginSignIn success")
                    try {
                        oneTapClientResult.launch(
                            IntentSenderRequest.Builder(result.pendingIntent.intentSender).build()
                        )
                        Timber.d("IntentSender launched")
                    } catch (e: Exception) {
                        Timber.e("Error launching IntentSender: ${e.message}")
                        hideLoading()
                    }
                }
                .addOnFailureListener(this) { e ->
                    Timber.e("beginSignIn failed: ${e.message}")
                    hideLoading()
                }
        }
    }
}