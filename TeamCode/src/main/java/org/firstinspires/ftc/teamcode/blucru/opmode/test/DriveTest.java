package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Drive test", group = "test")
public class DriveTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        addDrivetrain();
        dt.drivePower = 0.8;
        enableFTCDashboard();
    }

    @Override
    public void periodic() {
        if(gamepad1.right_stick_button) {
            dt.setPoseEstimate(new Pose2d(0,0,Math.PI/2));
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }

        dt.teleOpDrive(gamepad1);
        dt.drawPose();
    }
}
