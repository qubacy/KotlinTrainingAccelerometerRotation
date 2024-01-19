package com.qubacy.kotlintrainingaccelerometer

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import com.qubacy.kotlintrainingaccelerometer.databinding.ActivityMainBinding
import kotlin.math.PI
import kotlin.math.sin

class MainActivity : AppCompatActivity(), SensorEventListener {
    companion object {
        const val TAG = "MAIN_ACTIVITY"

        const val SENSOR_VECTOR_SIZE = 3
        const val FILTER_ALPHA = 0.25f

        const val PI_AS_DEGREES = 180

        const val NOT_INITIALIZED_LONG = -1L

        const val ACCELERATION_THRESHOLD = 0.4f // todo: should be set dynamically during a calibration stage;
        const val VELOCITY_THRESHOLD = 0.01f

        const val SENSOR_ITERATION_DURATION = 200L
    }

    private lateinit var mBinding: ActivityMainBinding
    private lateinit var mSensorManager: SensorManager

    private var mOriginAngles: FloatArray? = null

    private var mPrevVelocityVector: FloatArray = floatArrayOf(0f, 0f, 0f)

    private var mPrevFilteredRotationVector: FloatArray? = null
    private var mPrevFilteredAccelerationVector: FloatArray? = null

    private var mLastFilteredAngledRotationVector: FloatArray? = null

    private var mPrevAccelerationUpdateTime: Long = NOT_INITIALIZED_LONG

    private val mResultOffsetVector: FloatArray = floatArrayOf(0f, 0f, 0f)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        mBinding = ActivityMainBinding.inflate(layoutInflater)

        setContentView(mBinding.root)

        mSensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
    }

    override fun onStart() {
        super.onStart()

        val rotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        val linearAccelerationSensor = mSensorManager
            .getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        val magneticFieldForceSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)

        mSensorManager.registerListener(
            this, rotationSensor, SensorManager.SENSOR_DELAY_NORMAL)
        mSensorManager.registerListener(
            this, linearAccelerationSensor, SensorManager.SENSOR_DELAY_FASTEST)
        mSensorManager.registerListener(
            this, magneticFieldForceSensor, SensorManager.SENSOR_DELAY_NORMAL)
    }

    override fun onStop() {
        mSensorManager.unregisterListener(this)

        super.onStop()
    }

    override fun onSensorChanged(event: SensorEvent?) {
        when (event?.sensor?.type) {
            Sensor.TYPE_ROTATION_VECTOR -> processRotation(event.values!!)
            Sensor.TYPE_LINEAR_ACCELERATION -> processLinearAcceleration(event.values!!)
            Sensor.TYPE_MAGNETIC_FIELD -> processMagneticFieldForce(event.values!!)
        }
    }

    private fun processMagneticFieldForce(forceVector: FloatArray) {
        // todo: implement.. ?

        applyMagneticFieldForceVectorToUi(forceVector)
    }

    private fun applyMagneticFieldForceVectorToUi(forceVector: FloatArray) {
        mBinding.magneticFieldX.text =
            getString(R.string.magnetic_field_x_text).format(forceVector[0])
        mBinding.magneticFieldY.text =
            getString(R.string.magnetic_field_y_text).format(forceVector[1])
        mBinding.magneticFieldZ.text =
            getString(R.string.magnetic_field_z_text).format(forceVector[2])
    }

    // algo:
    // 1. get an initial Azimuth angle value (to set a relative North for the app);
    // 2. every iteration starts from getting current Angles (Azimuth, Pitch, Roll). then
    //    we detect which side of the device is faced Up, Down, Forward, Back, Right and Left;

    private fun processLinearAcceleration(accelerationVector: FloatArray) {
        if (mLastFilteredAngledRotationVector == null) return

        val updateTime = System.currentTimeMillis()

        if (mPrevAccelerationUpdateTime == NOT_INITIALIZED_LONG) {
            mPrevAccelerationUpdateTime = updateTime

            return
        }
//        if (mPrevAccelerationUpdateTime + SENSOR_ITERATION_DURATION > updateTime)
//            return

        val filteredAccelerationVector = filterAccelerationVector(accelerationVector)
        val siftedAccelerationVector = siftAccelerationVector(filteredAccelerationVector)
        val offsetVector = accelerationVectorToOffsetVector(siftedAccelerationVector, updateTime)
        val projectedOffsetVector = projectOffsetVectorWithAngles(
            offsetVector, mOriginAngles!!, mLastFilteredAngledRotationVector!!)

        mPrevAccelerationUpdateTime = updateTime

        // projectedOffsetVector.
        offsetVector.forEachIndexed { index, value -> mResultOffsetVector[index] += value }

        applyOffsetVectorToUi(mResultOffsetVector) // mResultOffsetVector
    }

    private fun projectOffsetVectorWithAngles(
        offsetVector: FloatArray,
        originAngleVector: FloatArray,
        angleVector: FloatArray
    ): FloatArray {
        val projectedOffsetVector = FloatArray(SENSOR_VECTOR_SIZE)

        for (i in 0 until SENSOR_VECTOR_SIZE) {
            val differenceAngle = (originAngleVector[i] - angleVector[i]) *
                    PI.toFloat() / PI_AS_DEGREES

            projectedOffsetVector[i] = sin(differenceAngle) * offsetVector[i]
        }

        return projectedOffsetVector
    }

    private fun siftAccelerationVector(accelerationVector: FloatArray): FloatArray {
        val siftedAccelerationVector = FloatArray(SENSOR_VECTOR_SIZE)

        for (i in 0 until SENSOR_VECTOR_SIZE) {
            siftedAccelerationVector[i] =
                if (accelerationVector[i] >= -ACCELERATION_THRESHOLD
                 && accelerationVector[i] <= ACCELERATION_THRESHOLD) 0f
                else accelerationVector[i]
        }

        return siftedAccelerationVector
    }

    private fun filterAccelerationVector(accelerationVector: FloatArray): FloatArray {
        mPrevFilteredAccelerationVector =
            filterSensorVector(FILTER_ALPHA, mPrevFilteredAccelerationVector, accelerationVector)

        return mPrevFilteredAccelerationVector!!
    }

    private fun accelerationVectorToOffsetVector(
        accelerationVector: FloatArray,
        updateTime: Long
    ): FloatArray {
        val interval = (updateTime - mPrevAccelerationUpdateTime) / 1000f
        val offsetVector = floatArrayOf(0f, 0f, 0f)

        for (i in 0 until SENSOR_VECTOR_SIZE) {
            if (accelerationVector[i] == 0f) {
                mPrevVelocityVector[i] = 0f

                continue
            }

            val accelerationToOffsetResult =
                accelerationToOffset(accelerationVector[i], mPrevVelocityVector[i], interval)

            if (accelerationToOffsetResult.curVelocity <= VELOCITY_THRESHOLD
             && accelerationToOffsetResult.curVelocity >= -VELOCITY_THRESHOLD
            ) {
                mPrevVelocityVector[i] = 0f

                continue
            }

            Log.d(TAG, "accelerationVectorToOffsetVector():" +
                    "axisIndex = $i;" +
                    " acceleration = ${accelerationVector[i]};" +
                    " curVelocity = ${accelerationToOffsetResult.curVelocity};" //+
                //" curVelocity = $curVelocity;" +
                //" distance = $distance;" +
                //" interval = $interval"
            )

            mPrevVelocityVector[i] = accelerationToOffsetResult.curVelocity
            offsetVector[i] = accelerationToOffsetResult.distance
        }

        return offsetVector
    }

    data class AccelerationToOffsetResult(
        val curVelocity: Float,
        val distance: Float
    )

    private fun accelerationToOffset(
        acceleration: Float,
        prevVelocity: Float,
        interval: Float
    ): AccelerationToOffsetResult {
        val curVelocity = prevVelocity + acceleration * interval
        val distance = prevVelocity * interval + 0.5f * acceleration * interval * interval

        return AccelerationToOffsetResult(curVelocity, distance)
    }

    private fun applyOffsetVectorToUi(offsetVector: FloatArray) {
        mBinding.offsetX.text = getString(R.string.offset_x_text).format(offsetVector[0])
        mBinding.offsetY.text = getString(R.string.offset_y_text).format(offsetVector[1])
        mBinding.offsetZ.text = getString(R.string.offset_z_text).format(offsetVector[2])
    }

    // Azimuth (around Z): N = 0; S = -1, 1;
    // Pitch (around X): Parallel = 0; Sky = 0.5; Ground = -0.5;
    // Roll (around Y): Down = 0; Up = -1, 1;

    private fun processRotation(rotationVector: FloatArray) {
        val filteredRotationVector = filterRotationVector(rotationVector)
        mLastFilteredAngledRotationVector =
            rotationVectorToAngleRotationVector(filteredRotationVector)

        if (mOriginAngles == null)
            mOriginAngles = floatArrayOf(mLastFilteredAngledRotationVector!!.get(2), 0f, 0f)

        applyRotationVectorToUi(mLastFilteredAngledRotationVector!!)
    }

    private fun applyRotationVectorToUi(rotationVector: FloatArray) {
        mBinding.rotationAzimuth.text = getString(R.string.rotation_azimuth_text).format(rotationVector[2])
        mBinding.rotationPitch.text =  getString(R.string.rotation_pitch_text).format(rotationVector[1])
        mBinding.rotationRoll.text =  getString(R.string.rotation_roll_text).format(rotationVector[0])
    }

    private fun rotationVectorToAngleRotationVector(rotationVector: FloatArray): FloatArray {
        val angleRotationVector = FloatArray(SENSOR_VECTOR_SIZE)

        angleRotationVector[0] = rotationVector[0] * PI_AS_DEGREES
        angleRotationVector[1] = rotationVector[1] * PI_AS_DEGREES
        angleRotationVector[2] = rotationVector[2] * PI_AS_DEGREES

        return angleRotationVector
    }

    private fun filterRotationVector(rotationVector: FloatArray): FloatArray {
        mPrevFilteredRotationVector =
            filterSensorVector(FILTER_ALPHA, mPrevFilteredRotationVector, rotationVector)

        return mPrevFilteredRotationVector!!
    }

    private fun filterSensorVector(
        alpha: Float,
        prevFilteredVector: FloatArray?,
        sensorVector: FloatArray
    ): FloatArray {
        val filteredVector = FloatArray(SENSOR_VECTOR_SIZE)

        if (prevFilteredVector == null) {
            filteredVector[0] = (1 - alpha) * sensorVector[0]
            filteredVector[1] = (1 - alpha) * sensorVector[1]
            filteredVector[2] = (1 - alpha) * sensorVector[2]

        } else {
            filteredVector[0] = alpha * prevFilteredVector[0] + (1 - alpha) * sensorVector[0]
            filteredVector[1] = alpha * prevFilteredVector[1] + (1 - alpha) * sensorVector[1]
            filteredVector[2] = alpha * prevFilteredVector[2] + (1 - alpha) * sensorVector[2]
        }

        return filteredVector
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {

    }
}