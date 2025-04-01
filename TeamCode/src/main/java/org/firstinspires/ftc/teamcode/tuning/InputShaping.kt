package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.EncoderGroup
import com.acmerobotics.roadrunner.ftc.EncoderRef
import com.acmerobotics.roadrunner.ftc.MidpointTimer
import com.acmerobotics.roadrunner.ftc.TuningFiles
import com.acmerobotics.roadrunner.ftc.TuningFiles.FileType
import com.acmerobotics.roadrunner.ftc.shouldFixVels
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.IMU
import kotlin.math.min

class MutableSignal(
    val times: MutableList<Double> = mutableListOf(),
    val values: MutableList<Double> = mutableListOf()
)
class InputShaping(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var POWER_PER_SEC = 0.1
        @JvmField
        var POWER_MAX = 0.9
    }
    private fun power(seconds: Double) = min(
        POWER_PER_SEC * seconds,
        POWER_MAX
    )

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)
        val imu = view.imu.get();

        val xRotation = 0.0 // enter the desired X rotation angle here.
        val yRotation = 0.0 // enter the desired Y rotation angle here.
        val zRotation = 0.0 // enter the desired Z rotation angle here.
        val hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        val orientationOnRobot : RevHubOrientationOnRobot = RevHubOrientationOnRobot(hubRotation);
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()
        require(view.perpEncs.isNotEmpty()) {
            "Only run this op mode if you're using dead wheels."
        }

        val data = object {
            val type = view.type
            val powers = view.motors.map { MutableSignal() }
            val voltages = MutableSignal()
            val forwardEncPositions = view.forwardEncs.map { MutableSignal() }
            val forwardEncVels = view.forwardEncs.map { MutableSignal() }
            val forwardEncFixVels = view.forwardEncs.map { shouldFixVels(view, it) }
            val heading = MutableSignal()
        }
        val data2 = object {
            val d = listOf(data);
        }

        waitForStart()

        val t = MidpointTimer()
        val lastTime = 0;
        var period = 0;
        var stop = false;
        while (opModeIsActive()) {
            // Retrieve Rotational Angles and Velocities
            val orientation = imu.robotYawPitchRollAngles
            if(!stop){
                for (i in view.motors.indices) {
                    val power = (period+1)*0.2;
                    view.motors[i].power = power

                    val s = data.powers[i]
                    s.times.add(t.addSplit())
                    s.values.add(power)
                }
                data2.d[period].voltages.values.add(view.voltageSensor.voltage)
                data2.d[period].voltages.times.add(t.addSplit())
                data2.d[period].heading.values.add(orientation.yaw)
                data2.d[period].heading.times.add(t.addSplit())

                val encTimes = view.encoderGroups.map {
                    it.bulkRead()
                    t.addSplit()
                }

                for (i in view.forwardEncs.indices) {
                    recordUnwrappedEncoderData(
                        view.encoderGroups,
                        encTimes,
                        view.forwardEncs[i],
                        data2.d[period].forwardEncPositions[i],
                        data2.d[period].forwardEncVels[i]
                    )
                }
            }
            if(gamepad1.x){
                stop = true;
            }
            if (gamepad1.y) {
                stop = false;
                period += 1;
                telemetry.addData("Yaw", "Resetting\n")
                imu.resetYaw()
            }
        }

        for (m in view.motors) {
            m.power = 0.0
        }

        TuningFiles.save(FileType.ACCEL, data)
    }
}
private fun recordUnwrappedEncoderData(gs: List<EncoderGroup>, ts: List<Double>, er: EncoderRef, ps: MutableSignal, vs: MutableSignal) {
    val t = ts[er.groupIndex]
    val e = gs[er.groupIndex].unwrappedEncoders[er.index]
    val pv = e.getPositionAndVelocity()

    ps.times.add(t)
    ps.values.add(pv.position.toDouble())

    if (pv.velocity != null) {
        vs.times.add(t)
        vs.values.add(pv.velocity!!.toDouble())
    }
}
