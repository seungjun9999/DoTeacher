package com.example.doteacher.ui.preference

import androidx.activity.viewModels
import com.example.doteacher.R
import com.example.doteacher.databinding.ActivityPreferenceBinding
import com.example.doteacher.ui.base.BaseActivity
import com.example.doteacher.ui.preference.viewmodel.PreferenceViewModel
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class PreferenceActivity : BaseActivity<ActivityPreferenceBinding>(R.layout.activity_preference) {

    private val viewModel: PreferenceViewModel by viewModels()

    override fun init() {
        supportFragmentManager.beginTransaction()
            .replace(R.id.fragment_container, PreferenceFragment())
            .commit()
    }
}