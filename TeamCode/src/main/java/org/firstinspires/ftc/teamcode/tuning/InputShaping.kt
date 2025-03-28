package org.firstinspires.ftc.teamcode.tuning

import android.os.Environment
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.EncoderGroup
import com.acmerobotics.roadrunner.ftc.EncoderRef
import com.acmerobotics.roadrunner.ftc.MidpointTimer
import com.acmerobotics.roadrunner.ftc.TuningFiles
import com.acmerobotics.roadrunner.ftc.TuningFiles.FileType
import com.acmerobotics.roadrunner.ftc.shouldFixVels
import com.fasterxml.jackson.core.JsonFactory
import com.fasterxml.jackson.databind.ObjectMapper
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.RobotLog
import java.io.File
import java.io.FileWriter
import java.io.IOException
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
    fun power(seconds: Double) = min(
        POWER_PER_SEC * seconds,
        POWER_MAX
    )

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)
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
        }

        waitForStart()

        val t = MidpointTimer()
        while (opModeIsActive()) {
            for (i in view.motors.indices) {
                val power = power(t.seconds())
                view.motors[i].power = power

                val s = data.powers[i]
                s.times.add(t.addSplit())
                s.values.add(power)
            }

            data.voltages.values.add(view.voltageSensor.voltage)
            data.voltages.times.add(t.addSplit())

            val encTimes = view.encoderGroups.map {
                it.bulkRead()
                t.addSplit()
            }

            for (i in view.forwardEncs.indices) {
                recordUnwrappedEncoderData(
                    view.encoderGroups,
                    encTimes,
                    view.forwardEncs[i],
                    data.forwardEncPositions[i],
                    data.forwardEncVels[i]
                )
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
