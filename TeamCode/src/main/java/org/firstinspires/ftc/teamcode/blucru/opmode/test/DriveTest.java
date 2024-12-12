package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Drive test", group = "test")
public class DriveTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        addDrivetrain();
        dt.setDrivePower(0.8);
        enableFTCDashboard();
        dt.setPoseEstimate(Globals.startPose);
    }

    @Override
    public void periodic() {
        if(gamepad1.right_stick_button) {
            dt.setPoseEstimate(Globals.startPose);
            gamepad1.rumble(150);
        }

        dt.teleOpDrive(gamepad1);
        dt.drawPose();
    }

    @Override
    public void end() {
        Log.i("DriveTest", "set start pose to: " + dt.pose);
        Globals.startPose = dt.pose;
    }

    @Override
    public void telemetry() {
        telemetry.addData("dt.pose: ", dt.pose);
    }
}
