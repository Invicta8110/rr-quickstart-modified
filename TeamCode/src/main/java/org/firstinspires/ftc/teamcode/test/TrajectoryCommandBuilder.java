package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.stream.Collectors;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

public class TrajectoryCommandBuilder {
    @FunctionalInterface
    public interface TrajectoryCommandFactory {
        Command build(TimeTrajectory trajectory);
    }

    @FunctionalInterface
    public interface SequentialFactory {
        Sequential build(Iterable<Command> command);

        static SequentialFactory sequence(Command command, SequentialFactory factory) {
            return (c) -> new Sequential(factory.build(c), command);
        }
    }

    private TrajectoryCommandFactory factory;
    private TrajectoryBuilder builder;
    private SequentialFactory continuation;

    public TrajectoryCommandBuilder(
            TrajectoryCommandFactory factory,
            TrajectoryBuilder builder,
            SequentialFactory continuation
    ) {
        this.factory = factory;
        this.builder = builder;
        this.continuation = continuation;
    }

    public Command build() {
        return this.continuation.build(
                builder.build()
                        .stream()
                        .map( traj -> new TimeTrajectory(traj.path, new TimeProfile(traj.profile.baseProfile)) )
                        .map( factory::build )
                        .collect(Collectors.toList())
        );
    }

    public TrajectoryCommandBuilder stopAndAdd(Command command) {
        return new TrajectoryCommandBuilder(
                this.factory,
                this.builder,
                SequentialFactory.sequence(command, this.continuation)
        );
    }

    public TrajectoryCommandBuilder setTangent(Rotation2d tangent) {
        builder = builder.setTangent(tangent);
        return this;
    }

    public TrajectoryCommandBuilder setTangent(double tangent) {
        return this.setTangent(Rotation2d.exp(tangent));
    }

    public TrajectoryCommandBuilder setReversed(boolean reversed) {
        builder = builder.setReversed(reversed);
        return this;
    }

    public TrajectoryCommandBuilder splineTo(
            Vector2d point,
            Rotation2d tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        builder = builder.splineTo(point, tangent, velOverride, accelOverride);
        return this;
    }

    public TrajectoryCommandBuilder splineTo(Vector2d point, Rotation2d tangent) {
        return this.splineTo(point, tangent, null, null);
    }

    public TrajectoryCommandBuilder splineTo(
            Vector2d point,
            double tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        return this.splineTo(point, Rotation2d.exp(tangent), velOverride, accelOverride);
    }

    public TrajectoryCommandBuilder splineTo(Vector2d point, double tangent) {
        return this.splineTo(point, Rotation2d.exp(tangent));
    }

    public TrajectoryCommandBuilder splineToSplineHeading(
            Pose2d point,
            Rotation2d tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        builder = builder.splineToSplineHeading(point, tangent, velOverride, accelOverride);
        return this;
    }

    public TrajectoryCommandBuilder splineToSplineHeading(Pose2d point, Rotation2d tangent) {
        return this.splineToSplineHeading(point, tangent, null, null);
    }

    public TrajectoryCommandBuilder splineToSplineHeading(
            Pose2d point,
            double tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        return this.splineToSplineHeading(point, Rotation2d.exp(tangent), velOverride, accelOverride);
    }

    public TrajectoryCommandBuilder splineToSplineHeading(Pose2d point, double tangent) {
        return this.splineToSplineHeading(point, Rotation2d.exp(tangent));
    }


    public TrajectoryCommandBuilder splineToLinearHeading(
            Pose2d point,
            Rotation2d tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        builder = builder.splineToLinearHeading(point, tangent, velOverride, accelOverride);
        return this;
    }

    public TrajectoryCommandBuilder splineToLinearHeading(Pose2d point, Rotation2d tangent) {
        return this.splineToLinearHeading(point, tangent, null, null);
    }

    public TrajectoryCommandBuilder splineToLinearHeading(
            Pose2d point,
            double tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        return this.splineToLinearHeading(point, Rotation2d.exp(tangent), velOverride, accelOverride);
    }

    public TrajectoryCommandBuilder splineToLinearHeading(Pose2d point, double tangent) {
        return this.splineToLinearHeading(point, Rotation2d.exp(tangent));
    }

    public TrajectoryCommandBuilder splineToConstantHeading(
            Vector2d point,
            Rotation2d tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        builder = builder.splineToConstantHeading(point, tangent, velOverride, accelOverride);
        return this;
    }

    public TrajectoryCommandBuilder splineToConstantHeading(Vector2d point, Rotation2d tangent) {
        return this.splineToConstantHeading(point, tangent, null, null);
    }

    public TrajectoryCommandBuilder splineToConstantHeading(
            Vector2d point,
            double tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        return this.splineToConstantHeading(point, Rotation2d.exp(tangent), velOverride, accelOverride);
    }

    public TrajectoryCommandBuilder splineToConstantHeading(Vector2d point, double tangent) {
        return this.splineToConstantHeading(point, Rotation2d.exp(tangent));
    }

    public TrajectoryCommandBuilder strafeTo(
            Vector2d point,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        builder = builder.strafeTo(point, velOverride, accelOverride);
        return this;
    }

    public TrajectoryCommandBuilder strafeTo(Vector2d point) {
        return this.strafeTo(point, null, null);
    }

    public TrajectoryCommandBuilder strafeToSplineHeading(
            Vector2d point,
            Rotation2d tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        builder = builder.strafeToSplineHeading(point, tangent, velOverride, accelOverride);
        return this;
    }

    public TrajectoryCommandBuilder strafeToSplineHeading(Vector2d point, Rotation2d tangent) {
        return this.strafeToSplineHeading(point, tangent, null, null);
    }

    public TrajectoryCommandBuilder strafeToSplineHeading(
            Vector2d point,
            double tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        return this.strafeToSplineHeading(point, Rotation2d.exp(tangent), velOverride, accelOverride);
    }

    public TrajectoryCommandBuilder strafeToSplineHeading(Vector2d point, double tangent) {
        return this.strafeToSplineHeading(point, Rotation2d.exp(tangent));
    }

    public TrajectoryCommandBuilder strafeToLinearHeading(
            Vector2d point,
            Rotation2d tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        builder = builder.strafeToLinearHeading(point, tangent, velOverride, accelOverride);
        return this;
    }

    public TrajectoryCommandBuilder strafeToLinearHeading(Vector2d point, Rotation2d tangent) {
        return this.strafeToLinearHeading(point, tangent, null, null);
    }

    public TrajectoryCommandBuilder strafeToLinearHeading(
            Vector2d point,
            double tangent,
            VelConstraint velOverride,
            AccelConstraint accelOverride
    ) {
        return this.strafeToLinearHeading(point, Rotation2d.exp(tangent), velOverride, accelOverride);
    }

    public TrajectoryCommandBuilder strafeToLinearHeading(Vector2d point, double tangent) {
        return this.strafeToLinearHeading(point, Rotation2d.exp(tangent));
    }

}
