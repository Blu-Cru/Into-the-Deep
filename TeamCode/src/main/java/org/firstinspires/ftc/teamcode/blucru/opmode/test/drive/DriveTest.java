package org.firstinspires.ftc.teamcode.blucru.opmode.test.drive;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.DriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Drive test", group = "test")
public class DriveTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        addDrivetrain();
        dt.setDrivePower(0.8);
        enableFTCDashboard();
//        dt.setPoseEstimate(DriveBase.startPose);
    }

    @Override
    public void onStart() {
        dt.setPoseEstimate(DriveBase.startPose);
    }

    @Override
    public void periodic() {
        if(stickyG1.left_bumper) {
            dt.setPoseEstimate(DriveBase.startPose);
            gamepad1.rumble(150);
        }

        if(stickyG1.right_stick_button) {
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }

        dt.teleOpDrive(gamepad1);
        dt.drawPose();
        DriveBase.startPose = dt.pose;
    }

    @Override
    public void end() {
        Log.i("DriveTest", "set start pose to: " + dt.pose);
//        DriveBase.startPose = dt.pose;
    }

    @Override
    public void telemetry() {
        telemetry.addData("dt.pose: ", dt.pose);
    }
}
