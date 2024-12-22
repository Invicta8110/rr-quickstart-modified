package org.firstinspires.ftc.teamcode.test;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.TimeTrajectory;

import dev.frozenmilk.mercurial.commands.Lambda;

public class FTCTest extends Lambda {
    TimeTrajectory traj;

    public FTCTest(TimeTrajectory traj) {
        super("FTCTest");

        this.traj = traj;
    }

    @NonNull
    @Override
    public String toString() {
        return "Trajectory With " + traj;
    }
}
