package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.VelConstraint
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection

object Params {
    // IMU orientation
    // TODO: fill in these values based on
    //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
    var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
        RevHubOrientationOnRobot.LogoFacingDirection.UP
    var usbFacingDirection: UsbFacingDirection = UsbFacingDirection.FORWARD

    // drive model parameters
    var inPerTick: Double = 1.0
    var lateralInPerTick: Double = inPerTick
    var trackWidthTicks: Double = 0.0

    // feedforward parameters (in tick units)
    var kS: Double = 0.0
    var kV: Double = 0.0
    var kA: Double = 0.0

    // path profile parameters (in inches)
    var maxWheelVel: Double = 50.0
    var minProfileAccel: Double = -30.0
    var maxProfileAccel: Double = 50.0

    // turn profile parameters (in radians)
    var maxAngVel: Double = Math.PI // shared with path
    var maxAngAccel: Double = Math.PI

    // path controller gains
    var axialGain: Double = 0.0
    var lateralGain: Double = 0.0
    var headingGain: Double = 0.0 // shared with turn

    var axialVelGain: Double = 0.0
    var lateralVelGain: Double = 0.0
    var headingVelGain: Double = 0.0 // shared with turn
}

val kinematics = MecanumKinematics(
    Params.inPerTick * Params.trackWidthTicks,
    Params.inPerTick / Params.lateralInPerTick
)

val defaultTurnConstraints = TurnConstraints(
    Params.maxAngVel, -Params.maxAngAccel, Params.maxAngAccel
)

val defaultVelConstraint: VelConstraint = MinVelConstraint(
    listOf(
        kinematics.WheelVelConstraint(Params.maxWheelVel),
        AngularVelConstraint(Params.maxAngVel)
    )
)

val defaultAccelConstraint: AccelConstraint =
    ProfileAccelConstraint(Params.minProfileAccel, Params.maxProfileAccel)