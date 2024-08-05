package com.example.doteacher.ui.preference

import android.content.Intent
import android.graphics.Color
import android.widget.TextView
import android.widget.Toast
import androidx.activity.viewModels
import androidx.lifecycle.viewModelScope
import com.example.doteacher.R
import com.example.doteacher.databinding.ActivityPreferenceBinding
import com.example.doteacher.ui.base.BaseActivity
import com.example.doteacher.ui.main.MainActivity
import com.example.doteacher.ui.preference.viewmodel.PreferenceViewModel
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.launch
import timber.log.Timber

@AndroidEntryPoint
class PreferenceActivity : BaseActivity<ActivityPreferenceBinding>(R.layout.activity_preference) {

    private val selectedPreferences = mutableSetOf<String>()
    private val viewModel: PreferenceViewModel by viewModels()

    override fun init() {
        clickEventListener()
        observePreferencesUpdate()
        loadUserPreferences()
    }

    private fun loadUserPreferences() {
        viewModel.loadUserPreferences()
        observeUserPreferences()
    }

    private fun observeUserPreferences() {
        viewModel.userPreferences.observe(this) { preferences ->
            selectedPreferences.clear()
            selectedPreferences.addAll(preferences)
            updatePreferenceUI()
        }
    }

    private fun updatePreferenceUI() {
        binding.apply {
            choice1.setTextColor(if (selectedPreferences.contains(choice1.text.toString())) Color.RED else Color.BLACK)
            choice2.setTextColor(if (selectedPreferences.contains(choice2.text.toString())) Color.RED else Color.BLACK)
            choice3.setTextColor(if (selectedPreferences.contains(choice3.text.toString())) Color.RED else Color.BLACK)
        }
    }

    private fun clickEventListener() {
        binding.apply {
            choice1.setOnClickListener { togglePreference(choice1) }
            choice2.setOnClickListener { togglePreference(choice2) }
            choice3.setOnClickListener { togglePreference(choice3) }

            btnNext.setOnClickListener {
                if (selectedPreferences.isNotEmpty()) {
                    viewModel.updateUserPreferences(selectedPreferences.toList())
                } else {
                    Toast.makeText(this@PreferenceActivity, "취향을 하나 이상 선택해주세요.", Toast.LENGTH_SHORT)
                        .show()
                }
            }
        }
    }

    private fun observePreferencesUpdate() {
        viewModel.preferencesUpdated.observe(this) { success ->
            if (success) {
                Timber.d("Preferences updated, moving to MainActivity")
                Timber.d("Updated preferences: ${SingletonUtil.user?.preferences} , ${SingletonUtil.user}")

                val intent = Intent(this@PreferenceActivity, MainActivity::class.java)
                startActivity(intent)
                finish()

            } else {
                Toast.makeText(this, "취향 업데이트에 실패했습니다. 다시 시도해주세요.", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun togglePreference(textView: TextView) {
        val preference = textView.text.toString()
        if (selectedPreferences.contains(preference)) {
            selectedPreferences.remove(preference)
            textView.setTextColor(Color.BLACK)
        } else {
            selectedPreferences.add(preference)
            textView.setTextColor(Color.RED)
        }
        Timber.d("Selected preferences: $selectedPreferences")
    }
}