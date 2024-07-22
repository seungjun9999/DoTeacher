package com.example.doteacher.ui.home

import android.view.View
import androidx.core.view.isVisible
import androidx.fragment.app.viewModels
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentHomeBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint


@AndroidEntryPoint
class HomeFragment : BaseFragment<FragmentHomeBinding>(R.layout.fragment_home) {
    private val homeViewModel : HomeViewModel by viewModels()
    private lateinit var homeAdapter: HomeAdapter

    private val viewModel: HomeViewModel by viewModels()



    override fun initView() {
        initData()
    }

    override fun initViewCreated(){
        clickEventListener()
    }

    override fun onResume() {
        super.onResume()
        initData()
    }


    private fun initData(){
        binding.userData = SingletonUtil.user
    }

    //클릭 이벤트
    private fun clickEventListener(){
        binding.menu.setOnClickListener {
            binding.homeMenu.bringToFront()
            animateView(binding.homeMenu,-binding.homeMenu.width.toFloat(),0f)
        }
    }

    private fun animateView(view: View, startTranslationX: Float, endTranslationX: Float) {
        view.translationX = startTranslationX
        view.isVisible = true
        view.animate()
            .translationX(endTranslationX)
            .setDuration(Companion.ANIMATION_DURATION)
            .setListener(null)
            .start()
    }

    companion object {
        private const val ANIMATION_DURATION = 300L
    }
}