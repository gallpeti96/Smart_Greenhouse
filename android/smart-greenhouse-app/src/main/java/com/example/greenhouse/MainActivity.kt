package com.example.greenhouse

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import androidx.fragment.app.Fragment
import com.example.greenhouse.fragments.HomeFragment
import com.example.greenhouse.fragments.PlantsFragment
import com.example.greenhouse.fragments.SettingsFragment
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val homefragment = HomeFragment()
        val plantsfragment = PlantsFragment()
        val settingsfragment = SettingsFragment()

        makeFragment(homefragment)

        bottomnavbar.setOnNavigationItemSelectedListener {
            when(it.itemId)
            {
                R.id.ic_home -> makeFragment(homefragment)
                R.id.ic_plant -> makeFragment(plantsfragment)
                R.id.ic_settings -> makeFragment(settingsfragment)
            }
            true
        }

    }

    private fun makeFragment(fragment: Fragment) =
        supportFragmentManager.beginTransaction().apply {
            replace(R.id.fl_wrapper, fragment)
            commit()
        }
}
