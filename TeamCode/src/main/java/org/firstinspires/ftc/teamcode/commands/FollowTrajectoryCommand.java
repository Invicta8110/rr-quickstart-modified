package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import dev.frozenmilk.mercurial.commands.Lambda;

public class FollowTrajectoryCommand extends Lambda {
    TimeTrajectory traj;
    MecanumDrive robot;
    double generalTime, startTime;

    public FollowTrajectoryCommand(TimeTrajectory traj, MecanumDrive robot) {
        super("TrajectoryCommand");
        this.traj = traj;
        this.robot = robot;
    }

    @Override
    public void initialise() {
        generalTime = 0;
        startTime = System.nanoTime() * 1e-9;
    }

    @Override
    public void execute() {
        generalTime = (System.nanoTime() * 1e-9) - startTime;

        Pose2dDual<Time> txWorldTarget = traj.get(generalTime);
        robot.targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = robot.updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        )
                .compute(txWorldTarget, robot.pose, robotVelRobot);
        robot.driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = robot.kinematics.inverse(command);
        double voltage = robot.voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        robot.mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightBack.setPower(rightBackPower);
        robot.rightFront.setPower(rightFrontPower);
    }

    @Override
    public boolean finished() {
        return generalTime >= traj.duration;
    }

    @Override
    public void end(boolean b) {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
    }

    @NonNull
    @Override
    public String toString() {
        return "TrajectoryCommand";
    }
}
