package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name="Drive To Position Test", group="test")
public class DriveToPositionTest extends BluLinearOpMode {
    public static double targetX = 0, targetY = 0, targetHeading = Math.toRadians(90);
    public static double power = 0.5;

    String mode = "driver control";

    public void initialize() {
        addDrivetrain();
        enableFTCDashboard();

        targetX = dt.getPoseEstimate().getX();
        targetY = dt.getPoseEstimate().getY();
        targetHeading = dt.getPoseEstimate().getHeading();
    }

    public void periodic() {
        dt.updateTranslationPID();
        dt.drivePower = power;

        if(gamepad1.a) {
            mode = "driver control";
        }

        if(gamepad1.b) {
            mode = "drive to position";
        }

        if(mode.equals("driver control")) {
            if(gamepad1.right_stick_button) {
                dt.resetHeading(Math.toRadians(90));
                gamepad1.rumble(100);
            }

            dt.teleOpDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        } else if(mode.equals("drive to position")) {
            dt.pidTo(new Pose2d(targetX, targetY, targetHeading));
        }
    }

    public void telemetry() {
        telemetry.addData("mode", mode);
        telemetry.addData("target x", targetX);
        telemetry.addData("target y", targetY);
        telemetry.addData("target heading", targetHeading);
        dt.ftcDashDrawPose();
        telemetry.addData("current x", dt.getPoseEstimate().getX());
        telemetry.addData("current y", dt.getPoseEstimate().getY());
        telemetry.addData("current heading", dt.getPoseEstimate().getHeading());
    }
}
