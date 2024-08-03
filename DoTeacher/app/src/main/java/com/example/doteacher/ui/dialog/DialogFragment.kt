package com.example.doteacher.ui.dialog

import android.annotation.SuppressLint
import android.app.Dialog
import android.content.Context
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.LinearLayout
import androidx.core.content.ContextCompat
import androidx.fragment.app.DialogFragment
import androidx.viewpager2.widget.ViewPager2
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentDialogBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class DialogFragment : DialogFragment() {

    private lateinit var viewPager: ViewPager2
    private lateinit var indicator: LinearLayout
    private lateinit var binding: FragmentDialogBinding

    interface DialogListener {
        fun onDialogClosed(neverShowAgain: Boolean)
    }

    private var listener: DialogListener? = null

    @SuppressLint("MissingInflatedId")
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentDialogBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        viewPager = binding.viewPager
        indicator = binding.indicator

        val imageList = listOf(
            R.drawable.dosunbird,
            R.drawable.ex_cute_cat,
            R.drawable.ex_sky
        )

        val adapter = DialogAdapter(imageList)
        viewPager.adapter = adapter

        setupIndicators(imageList.size)
        setCurrentIndicator(0)

        viewPager.registerOnPageChangeCallback(object : ViewPager2.OnPageChangeCallback() {
            override fun onPageSelected(position: Int) {
                setCurrentIndicator(position)
            }
        })

        binding.btnCancelDialog.setOnClickListener {
            listener?.onDialogClosed(binding.checkBox.isChecked)
            dismiss()
        }
    }

    override fun onCreateDialog(savedInstanceState: Bundle?): Dialog {
        val dialog = super.onCreateDialog(savedInstanceState)
        dialog.window?.setBackgroundDrawableResource(android.R.color.transparent)
        return dialog
    }

    private fun setupIndicators(count: Int) {
        val indicators = arrayOfNulls<ImageView>(count)
        val layoutParams: LinearLayout.LayoutParams = LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT
        )
        layoutParams.setMargins(8, 0, 8, 0)
        for (i in indicators.indices) {
            indicators[i] = ImageView(requireContext())
            indicators[i]?.apply {
                setImageDrawable(ContextCompat.getDrawable(requireContext(), R.drawable.indicator_inactive))
                this.layoutParams = layoutParams
            }
            indicator.addView(indicators[i])
        }
    }

    private fun setCurrentIndicator(index: Int) {
        val childCount = indicator.childCount
        for (i in 0 until childCount) {
            val imageView = indicator.getChildAt(i) as ImageView
            if (i == index) {
                imageView.setImageDrawable(ContextCompat.getDrawable(requireContext(), R.drawable.indicator_active))
            } else {
                imageView.setImageDrawable(ContextCompat.getDrawable(requireContext(), R.drawable.indicator_inactive))
            }
        }
    }

    override fun onStart() {
        super.onStart()
        dialog?.window?.setLayout(380.dpToPx(requireContext()), 700.dpToPx(requireContext()))
    }

    fun Int.dpToPx(context: Context): Int =
        (this * context.resources.displayMetrics.density).toInt()

    fun setDialogListener(listener: DialogListener) {
        this.listener = listener
    }
}