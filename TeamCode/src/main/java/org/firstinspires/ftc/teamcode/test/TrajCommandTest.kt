package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.NullAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential

@Autonomous
class TrajCommandTest : LinearOpMode() {
    override fun runOpMode() {
        val seq = commandBuilder(Pose2d(0.0, 0.0, 0.0))
            .splineTo(Vector2d(12.0, 12.0), Math.toRadians(90.0))
            .strafeTo(Vector2d(24.0, 24.0))
            .stopAndAdd(Lambda.of("1", NullAction()))
            .strafeTo(Vector2d(36.0, 36.0))
            .stopAndAdd(Lambda.of("2", NullAction()))

        waitForStart()

        println(seq.build())
    }
}

fun Lambda.Companion.of(a: Action): Lambda {
    return Lambda("Anonymous").setFinish { a.run(TelemetryPacket()) }
}

fun Lambda.Companion.of(label: String, a: Action): Lambda = Lambda(label).setFinish { a.run(TelemetryPacket()) }

fun commandBuilder(beginPose: Pose2d?): TrajectoryCommandBuilder {
    return TrajectoryCommandBuilder(
        { traj: TimeTrajectory? -> FTCTest(traj) },
        TrajectoryBuilder(
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25,
                    0.1,
                    1e-2
                )
            ),
            beginPose!!,
            0.0,
            defaultVelConstraint,
            defaultAccelConstraint
        ),
        { seq -> Sequential(seq) }
    )
}
