package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FiveSpecimenConfig extends AutoConfig {
    int scoreCount;

    enum State {
        
    }

    public FiveSpecimenConfig() {
        scoreCount = 0;
    }

    @Override
    public void build() {

    }

    @Override
    public void start() {

    }

    @Override
    public void telemetry() {

    }

    @Override
    public Pose2d getStartPose() {
        return Globals.mapPose(10, -64, 90);
    }
}
