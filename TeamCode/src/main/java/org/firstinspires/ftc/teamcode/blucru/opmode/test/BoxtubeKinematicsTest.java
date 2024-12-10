package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Boxtube Kinematics Test", group = "Test")
public class BoxtubeKinematicsTest extends BluLinearOpMode {
    enum State {
        RETRACT,
        IVK
    }
    public static double x = 5, y = 5, angle = 0;
    Pose2d targetPose = new Pose2d(x, y, angle);
    State state = State.RETRACT;

    @Override
    public void initialize() {
        addArm();
        addWrist();
        addClamp();
        addPivot();
        addExtension();
        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void periodic() {

    }

    @Override
    public void telemetry() {

    }
}
