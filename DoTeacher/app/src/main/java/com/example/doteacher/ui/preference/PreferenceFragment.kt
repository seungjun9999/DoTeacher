package com.example.doteacher.ui.preference

import android.animation.AnimatorSet
import android.animation.ObjectAnimator
import android.content.Intent
import android.os.Handler
import android.os.Looper
import android.view.View
import android.view.animation.OvershootInterpolator
import android.widget.Toast
import androidx.fragment.app.activityViewModels
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentPreferenceBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.main.MainActivity
import com.example.doteacher.ui.preference.viewmodel.PreferenceViewModel
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

@AndroidEntryPoint
class PreferenceFragment : BaseFragment<FragmentPreferenceBinding>(R.layout.fragment_preference) {

    private val selectedPreferences = mutableSetOf<String>()
    private val viewModel: PreferenceViewModel by activityViewModels()
    private lateinit var preferenceAdapter: PreferenceAdapter

    private val preferenceList = listOf(
        Pair(R.drawable.daesundang, "낭만주의"),
        Pair(R.drawable.paris_load, "인상주의"),
        Pair(R.drawable.dolpeople, "사실주의"),
        Pair(R.drawable.gosung, "풍경화"),
        Pair(R.drawable.opera, "인물화"),
        Pair(R.drawable.sunflower, "사물화"),
        Pair(R.drawable.gosansu, "동양 고전"),
        Pair(R.drawable.bubu, "르네상스"),
        Pair(R.drawable.piano, "모더니즘")
    )

    override fun initView() {
        init()
    }

    override fun onResume() {
        super.onResume()
        hideBottomNavigation()
        loadUserPreferences()
    }

    override fun onPause() {
        super.onPause()
        showBottomNavigation()
    }

    private fun init() {
        setupRecyclerView()
        setupSelectedImageView()
        clickEventListener()
        observePreferencesUpdate()
        setInitialImage()
    }

    private fun hideBottomNavigation() {
        (activity as? MainActivity)?.hideBottomNavigation()
    }

    private fun showBottomNavigation() {
        (activity as? MainActivity)?.showBottomNavigation()
    }

    private fun setupRecyclerView() {
        preferenceAdapter = PreferenceAdapter(preferenceList) { imageResId, preferenceName ->
            binding.selectedImageView.setImageResource(imageResId)
            binding.selectedImageView.tag = preferenceName
            updateSelectedImageBorder(preferenceName)
        }
        binding.preferenceRecyclerView.adapter = preferenceAdapter
    }

    private fun loadUserPreferences() {
        viewModel.loadUserPreferences()
        observeUserPreferences()
    }

    private fun setInitialImage() {
        if (preferenceList.isNotEmpty()) {
            val (imageResId, preferenceName) = preferenceList.first()
            binding.selectedImageView.setImageResource(imageResId)
            binding.selectedImageView.tag = preferenceName
            updateSelectedImageBorder(preferenceName)
        }
    }

    private fun observeUserPreferences() {
        viewModel.userPreferences.observe(viewLifecycleOwner) { preferences ->
            selectedPreferences.clear()
            selectedPreferences.addAll(preferences)
            preferenceAdapter.updateSelectedItems(selectedPreferences)
            updateAllSelectedImageBorders()
        }
    }

    private fun updateAllSelectedImageBorders() {
        val currentPreference = binding.selectedImageView.tag as? String
        if (currentPreference != null) {
            updateSelectedImageBorder(currentPreference)
        }
        preferenceAdapter.notifyDataSetChanged()
    }

    private fun clickEventListener() {
        binding.btnNext.setOnClickListener {
            if (selectedPreferences.isNotEmpty()) {
                viewModel.updateUserPreferences(selectedPreferences.toList())
            } else {
                Toast.makeText(requireContext(), "취향을 하나 이상 선택해주세요.", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun observePreferencesUpdate() {
        viewModel.preferencesUpdated.observe(viewLifecycleOwner) { success ->
            if (success) {
                Timber.d("Preferences updated, moving to MainActivity")
                Timber.d("Updated preferences: ${SingletonUtil.user?.preferences} , ${SingletonUtil.user}")

                val intent = Intent(requireContext(), MainActivity::class.java)
                startActivity(intent)
                requireActivity().finish()
            } else {
                Toast.makeText(requireContext(), "취향 업데이트에 실패했습니다. 다시 시도해주세요.", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun updateSelectedImageBorder(preference: String) {
        if (selectedPreferences.contains(preference)) {
            binding.selectedImageBorderbig.visibility = View.VISIBLE
        } else {
            binding.selectedImageBorderbig.visibility = View.INVISIBLE
        }
    }

    private fun togglePreference(preference: String) {
        if (selectedPreferences.contains(preference)) {
            selectedPreferences.remove(preference)
        } else {
            selectedPreferences.add(preference)
        }
        updateSelectedImageBorder(preference)
        preferenceAdapter.updateSelectedItems(selectedPreferences)
        Timber.d("Selected preferences: $selectedPreferences")
    }

    private fun setupSelectedImageView() {
        var lastClickTime = 0L
        binding.selectedImageView.setOnClickListener {
            val clickTime = System.currentTimeMillis()
            if (clickTime - lastClickTime < 300) {  // 더블 클릭으로 간주할 시간 간격 (300ms)
                val preferenceName = it.tag as? String ?: return@setOnClickListener
                val isSelected = selectedPreferences.contains(preferenceName)
                togglePreference(preferenceName)
                if (isSelected) {
                    showUnlikeAnimation()
                } else {
                    showLikeAnimation()
                }
            }
            lastClickTime = clickTime
        }
    }

    private fun showUnlikeAnimation() {
        binding.likeImageView.apply {
            setImageResource(R.drawable.like)
            visibility = View.VISIBLE
            alpha = 0f
            scaleX = 1.2f
            scaleY = 1.2f
        }

        val fadeIn = ObjectAnimator.ofFloat(binding.likeImageView, "alpha", 0f, 1f)
        val scaleX = ObjectAnimator.ofFloat(binding.likeImageView, "scaleX", 1.2f, 0.5f)
        val scaleY = ObjectAnimator.ofFloat(binding.likeImageView, "scaleY", 1.2f, 0.5f)

        AnimatorSet().apply {
            playTogether(fadeIn, scaleX, scaleY)
            duration = 300
            interpolator = OvershootInterpolator()
            start()
        }

        Handler(Looper.getMainLooper()).postDelayed({
            val fadeOut = ObjectAnimator.ofFloat(binding.likeImageView, "alpha", 1f, 0f)
            fadeOut.duration = 300
            fadeOut.start()
        }, 700)
    }

    private fun showLikeAnimation() {
        binding.likeImageView.apply {
            visibility = View.VISIBLE
            alpha = 0f
            scaleX = 0.5f
            scaleY = 0.5f
        }

        val fadeIn = ObjectAnimator.ofFloat(binding.likeImageView, "alpha", 0f, 1f)
        val scaleX = ObjectAnimator.ofFloat(binding.likeImageView, "scaleX", 0.5f, 1.2f)
        val scaleY = ObjectAnimator.ofFloat(binding.likeImageView, "scaleY", 0.5f, 1.2f)

        AnimatorSet().apply {
            playTogether(fadeIn, scaleX, scaleY)
            duration = 300
            interpolator = OvershootInterpolator()
            start()
        }

        Handler(Looper.getMainLooper()).postDelayed({
            val fadeOut = ObjectAnimator.ofFloat(binding.likeImageView, "alpha", 1f, 0f)
            fadeOut.duration = 300
            fadeOut.start()
        }, 700)
    }
}
