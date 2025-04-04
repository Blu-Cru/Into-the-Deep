package org.firstinspires.ftc.teamcode.blucru.opmode.test.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "test")
public class DrivePIDTest extends BluLinearOpMode {
    public static double targetX = 0, targetY = 0, targetHeading = 0;

    @Override
    public void initialize() {
        addDrivetrain();
        enableFTCDashboard();
        dt.setDrivePower(1.0);
    }

    @Override
    public void onStart() {
        targetX = dt.pose.getX();
        targetY = dt.pose.getY();
        targetHeading = dt.heading;
    }

    @Override
    public void periodic() {
        dt.updatePID();

        if(gamepad1.a) {
            dt.pidTo(new Pose2d(targetX, targetY, targetHeading));
        } else {
            dt.idle();

            if(gamepad1.right_stick_button) {
                dt.setHeading(Math.PI/2);
            }

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rot = -gamepad1.right_stick_x;

            Pose2d drivePose = new Pose2d(x, y, rot);

            dt.driveFieldCentric(drivePose);
        }

        dt.drawPose();
    }

    @Override
    public void telemetry() {
        telemetry.addData("Target heading", targetHeading);
    }
}
