package com.qubacy.kotlintrainingaccelerometer

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import com.qubacy.kotlintrainingaccelerometer.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity(), SensorEventListener {
    companion object {
        const val TAG = "MAIN_ACTIVITY"

        const val ROTATION_SENSOR_UPDATE_TIMING = 100000
    }

    private lateinit var mBinding: ActivityMainBinding
    private lateinit var mSensorManager: SensorManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        mBinding = ActivityMainBinding.inflate(layoutInflater)

        setContentView(mBinding.root)

        mSensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
    }

    override fun onStart() {
        super.onStart()

        val rotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        mSensorManager.registerListener(this, rotationSensor, ROTATION_SENSOR_UPDATE_TIMING)
    }

    override fun onStop() {
        mSensorManager.unregisterListener(this)

        super.onStop()
    }

    override fun onSensorChanged(event: SensorEvent?) {
        when (event?.sensor?.type) {
            Sensor.TYPE_ROTATION_VECTOR -> processRotationVector(event.values!!)
        }
    }

    // Azimuth (around Z): N = 0; S = -1, 1;
    // Pitch (around X): Parallel = 0; Sky = 0.5; Ground = -0.5;
    // Roll (around Y): Down = 0; Up = -1, 1;

    private fun processRotationVector(rotationVector: FloatArray) {
        // todo: filter the inputs..



        mBinding.rotationAzimuth.text = getString(R.string.rotation_azimuth_text).format(rotationVector[2])
        mBinding.rotationPitch.text =  getString(R.string.rotation_pitch_text).format(rotationVector[0])
        mBinding.rotationRoll.text =  getString(R.string.rotation_roll_text).format(rotationVector[1])
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {

    }
}