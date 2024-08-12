package com.example.doteacher.ui.mypage

import android.os.Handler
import android.os.Looper
import androidx.navigation.fragment.findNavController
import androidx.viewpager2.widget.ViewPager2
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentMyPageBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber


@AndroidEntryPoint
class MyPageFragment : BaseFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {


    private lateinit var viewPager: ViewPager2
    private val sliderHandler = Handler(Looper.getMainLooper())
    private val sliderRunnable = Runnable { viewPager.currentItem = (viewPager.currentItem + 1) % 5 }

    override fun initView() {
        setupMyPageItems()
        initData()
        setupViewPager()
    }

    private fun initData(){
        binding.userData = SingletonUtil.user
    }

    private fun setupMyPageItems() {
        with(binding) {
            itemProfile.apply {
                imgMypage.setImageResource(R.drawable.set_profile) // 프로필 아이콘 리소스
                tvMypageText.text = "프로필"
                root.setOnClickListener {
                    findNavController().navigate(R.id.action_mypageFragment_to_profileFragment)
                }
            }
            itemPreferences.apply {
                imgMypage.setImageResource(R.drawable.set_edit) // 취향수정 아이콘 리소스
                tvMypageText.text = "취향수정"
                root.setOnClickListener {
                    findNavController().navigate(R.id.action_mypageFragment_to_preferenceActivity)
                }
            }
            itemGuide.apply {
                imgMypage.setImageResource(R.drawable.set_book) // 사용법 안내 아이콘 리소스
                tvMypageText.text = "사용법 안내"
            }
            itemSettings.apply {
                imgMypage.setImageResource(R.drawable.page_set) // 설정 아이콘 리소스
                tvMypageText.text = "설정"
                root.setOnClickListener{
                    Timber.d("setting button clicked")
                    findNavController().navigate(R.id.action_mypageFragment_to_settingsFragment)
                }
            }
        }
    }

    private fun setupViewPager() {
        viewPager = binding.viewpager
        val images = listOf(
            R.drawable.dosunbird,
            R.drawable.set_book,
            R.drawable.set_edit,
            R.drawable.set_profile,
            R.drawable.hwangso
        )

        val adapter = MyPageAdapter(images)
        viewPager.adapter = adapter

        viewPager.registerOnPageChangeCallback(object : ViewPager2.OnPageChangeCallback() {
            override fun onPageSelected(position: Int) {
                super.onPageSelected(position)
                sliderHandler.removeCallbacks(sliderRunnable)
                sliderHandler.postDelayed(sliderRunnable, 5000)
            }
        })
    }


    override fun onPause() {
        super.onPause()
        sliderHandler.removeCallbacks(sliderRunnable)
    }

    override fun onResume() {
        super.onResume()
        sliderHandler.postDelayed(sliderRunnable, 3000)
    }
}