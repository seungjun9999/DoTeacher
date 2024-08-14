import java.util.Properties

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.jetbrains.kotlin.android)
    id("com.google.gms.google-services")
    kotlin("kapt")
    id("kotlin-parcelize")
    id("com.google.dagger.hilt.android")
    id("androidx.navigation.safeargs.kotlin")
}


fun getLocalProperty(key: String): String {
    val properties = Properties()
    properties.load(project.rootProject.file("local.properties").inputStream())
    return properties.getProperty(key) ?: ""
}


android {
    namespace = "com.example.doteacher"
    compileSdk = 35
    compileSdkPreview = "VanillaIceCream"

    defaultConfig {
        applicationId = "com.example.doteacher"
        minSdk = 24
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        buildConfigField("String", "AWS_BUCKET_NAME", "\"${getLocalProperty("aws.bucket.name")}\"")
        buildConfigField("String", "AWS_ACCESS_KEY", "\"${getLocalProperty("aws.access.key")}\"")
        buildConfigField("String", "AWS_SECRET_KEY", "\"${getLocalProperty("aws.secret.key")}\"")

    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }

    buildFeatures {
        viewBinding = true
        buildConfig = true
        dataBinding = true
    }
}


dependencies {

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)
    implementation(libs.androidx.activity)
    implementation(libs.androidx.constraintlayout)
    implementation(libs.androidx.legacy.support.v4)
    implementation(libs.androidx.lifecycle.livedata.ktx)
    implementation(libs.androidx.fragment.ktx)
    implementation(libs.androidx.palette.ktx)
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    implementation(libs.androidx.lifecycle.viewmodel.ktx.v240)

    //Google service
    // Import the BoM for the Firebase platform
    implementation(platform(libs.firebase.bom))
    implementation(libs.firebase.database.ktx)
    // Add the dependency for the Firebase Authentication library
    // When using the BoM, you don't specify versions in Firebase library dependencies
    implementation(libs.firebase.auth.ktx)
    implementation(libs.firebase.analytics)
    // Also add the dependency for the Google Play services library and specify its version
    implementation(libs.play.services.auth)


    //timber
    implementation(libs.timber)

    // retrofit
    implementation(libs.retrofit)
    implementation(libs.converter.gson)
    implementation(libs.converter.scalars)


    //okHttpClient
    implementation(libs.okhttp)
    implementation(libs.logging.interceptor)

    // hilt
    implementation(libs.hilt.android)
    kapt(libs.hilt.android.compiler)

    // navigation
    implementation(libs.androidx.navigation.fragment.ktx)
    implementation(libs.androidx.navigation.ui.ktx)

    //circleimageView
    implementation (libs.circleimageview)

    // gilde
    implementation(libs.glide)
    annotationProcessor(libs.compiler)

    implementation (libs.androidx.core.ktx)

    implementation ("androidx.biometric:biometric:1.2.0-alpha05")

    implementation ("com.google.android.libraries.identity.googleid:googleid:1.1.0")
    implementation ("androidx.credentials:credentials:1.2.2")
    implementation ("androidx.credentials:credentials-play-services-auth:1.2.2")
    implementation("androidx.credentials:credentials-play-services-auth:1.5.0-alpha03")
    implementation("androidx.credentials:credentials:1.5.0-alpha03")

    //coroutines
    implementation (libs.kotlinx.coroutines.android)

    //datastore
    implementation (libs.androidx.datastore.preferences)

    //s3
    implementation (libs.aws.android.sdk.s3)

    implementation (libs.glide)
    annotationProcessor (libs.compiler)

    implementation ("com.journeyapps:zxing-android-embedded:4.2.0")

    implementation (libs.lottie)

}