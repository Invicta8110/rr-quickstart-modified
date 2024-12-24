package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.MecanumDrive

@Autonomous
@Mercurial.Attach
class TrajCommandTest : OpMode() {
    val robot by OpModeLazyCell { MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0)) }

    var seq by OpModeLazyCell { robot.trajectoryBuilder(robot.pose) }

    override fun init() {
        seq = seq.strafeTo(Vector2d(24.0, 24.0))
            .splineTo(Vector2d(-24.0, 12.0), Rotation2d.exp(Math.PI))
    }

    override fun start() {
        val commands = seq.build().map { robot.followTrajectoryCommand(it) }
        commands.forEach { it.schedule() }
    }

    override fun loop() {
        telemetry.addData("Pose", robot.pose)
    }
}
