package com.example.greenhouse

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.auth.FirebaseUser
import kotlinx.android.synthetic.main.activity_login.*
import android.util.Patterns
import android.widget.Toast

class Login : AppCompatActivity() {

    private lateinit var auth: FirebaseAuth

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_login)

        auth = FirebaseAuth.getInstance()

        logbutton.setOnClickListener {
            login()
        }

        notreg.setOnClickListener {
            startActivity(Intent(this, Register::class.java))
            finish()
        }
    }

    private fun login() {
        if (logemail.text.toString().isEmpty()) {
            logemail.error = "Please enter email."
            logemail.requestFocus()
            return
        }
        if (!Patterns.EMAIL_ADDRESS.matcher(logemail.text.toString()).matches()) {
            logemail.error = "Please enter a valid email."
            logemail.requestFocus()
            return
        }
        if (logpw.text.toString().isEmpty()) {
            logpw.error = "Please enter password."
            logpw.requestFocus()
            return
        }

        auth.signInWithEmailAndPassword(logemail.text.toString(), logpw.text.toString())
            .addOnCompleteListener(this) { task ->
                if (task.isSuccessful) {
                    val user = auth.currentUser
                    updateUI(user)
                } else {
                    updateUI(null)
                }
            }
    }

    public override fun onStart() {
        super.onStart()
        val currentUser = auth.currentUser
        updateUI(currentUser)
    }

    private fun updateUI(currentUser : FirebaseUser?){
        if(currentUser != null){
            startActivity(Intent(this, MainActivity::class.java))
        }else{
            Toast.makeText(baseContext, "Login failed.",
                Toast.LENGTH_SHORT).show()
        }
    }
}
