package org.firstinspires.ftc.teamcode.tuning

import android.os.Environment
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.MidpointTimer
import com.acmerobotics.roadrunner.ftc.TuningFiles
import com.acmerobotics.roadrunner.ftc.shouldFixVels
import com.fasterxml.jackson.core.JsonFactory
import com.fasterxml.jackson.databind.ObjectMapper
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import java.io.File
import java.io.FileWriter
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
        }

        for (m in view.motors) {
            m.power = 0.0
        }
        val s = File(Environment.getRootDirectory().path+"/inputShape/"+"input_shaping.json");
        s.createNewFile();
        ObjectMapper(JsonFactory())
            .writerWithDefaultPrettyPrinter()
            .writeValue(s, data)
        TuningFiles.save(TuningFiles.FileType.INPUT_SHAPING, data)
    }
}