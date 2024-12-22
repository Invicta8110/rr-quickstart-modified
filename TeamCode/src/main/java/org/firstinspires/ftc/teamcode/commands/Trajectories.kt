package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.groups.Sequential
import java.util.stream.Collectors

fun interface TrajectoryCommandFactory {
    fun build(trajectory: TimeTrajectory?): Command?
}

fun interface SequentialFactory {
    fun build(command: Iterable<Command?>?): Sequential

    companion object {
        fun sequence(command: Command, factory: SequentialFactory): SequentialFactory {
            return SequentialFactory { c: Iterable<Command?>? ->
                Sequential(
                    factory.build(c),
                    command
                )
            }
        }
    }
}

class TrajectoryCommandBuilder(
    private val factory: TrajectoryCommandFactory,
    private val beginPose: Pose2d,
    private var params: TrajectoryBuilderParams,
    private var baseVelConstraint: VelConstraint,
    private var baseAccelConstraint: AccelConstraint,
) {

    private var builder = TrajectoryBuilder(
        params,
        beginPose,
        0.0,
        baseVelConstraint,
        baseAccelConstraint
    )

    var size = 0;
    val commands: MutableList<Command> = mutableListOf()

    fun endTrajectory(): TrajectoryCommandBuilder {
        val built = builder.build()
        built.map { TimeTrajectory(it.path, TimeProfile(it.profile.baseProfile)) }
            .mapNotNull { factory.build(it) }
            .forEach { commands.add(it) }

        builder = TrajectoryBuilder(
            params,
            built.last().path.end(1).value(),
            0.0,
            baseVelConstraint,
            baseAccelConstraint,
            built.last().path.poseMap
        )

        return this
    }

    fun build(): Command {
        endTrajectory()

        return Sequential(commands)
    }

    fun stopAndAdd(command: Command): TrajectoryCommandBuilder {
        size += 1
        endTrajectory()
        commands += command
        return this
    }

    fun setTangent(tangent: Rotation2d): TrajectoryCommandBuilder {
        builder = builder.setTangent(tangent)
        return this
    }

    fun setTangent(tangent: Double): TrajectoryCommandBuilder {
        return this.setTangent(Rotation2d.exp(tangent))
    }

    fun setReversed(reversed: Boolean): TrajectoryCommandBuilder {
        builder = builder.setReversed(reversed)
        return this
    }

    @JvmOverloads
    fun splineTo(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1;
        builder = builder.splineTo(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun splineTo(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineTo(point, Rotation2d.exp(tangent), velOverride, accelOverride)
    }

    @JvmOverloads
    fun splineToSplineHeading(
        point: Pose2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1;
        builder = builder.splineToSplineHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun splineToSplineHeading(
        point: Pose2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineToSplineHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun splineToLinearHeading(
        point: Pose2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size +=1 ;
        builder = builder.splineToLinearHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun splineToLinearHeading(
        point: Pose2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineToLinearHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun splineToConstantHeading(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1;
        builder = builder.splineToConstantHeading(point, tangent, velOverride, accelOverride)
        return this
    }

@JvmOverloads
    fun splineToConstantHeading(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineToConstantHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun strafeTo(
        point: Vector2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1;
        builder = builder.strafeTo(point, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun strafeToSplineHeading(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.strafeToSplineHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun strafeToSplineHeading(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.strafeToSplineHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun strafeToLinearHeading(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1;
        builder = builder.strafeToLinearHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun strafeToLinearHeading(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.strafeToLinearHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }
}
