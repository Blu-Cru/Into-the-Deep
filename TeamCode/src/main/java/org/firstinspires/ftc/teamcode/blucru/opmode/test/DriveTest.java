package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Drive test", group = "test")
public class DriveTest extends BluLinearOpMode {
    double vert, horz, rotate;

    @Override
    public void initialize() {
        addDrivetrain();
        dt.drivePower = 0.8;
        enableFTCDashboard();
    }

    @Override
    public void periodic() {
        vert = -gamepad1.left_stick_y;
        horz = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) {
            dt.setPoseEstimate(new Pose2d(0,0,Math.PI/2));
            dt.resetHeading(Math.PI/2);
            gamepad1.rumble(150);
        }

        if(gamepad1.b) {
            if(gamepad1.left_bumper) {
//                    drivetrain.driveToDistanceToHeading(horz, vert, 10, Math.toRadians(180));
            } else {
                dt.driveToHeadingScaled(horz, vert, Math.toRadians(180));
            }
        } else if (gamepad1.x) {
            if(gamepad1.left_bumper) {
//                    drivetrain.driveToDistanceToHeading(horz, vert, 10, 0);
            } else {
                dt.driveToHeadingScaled(horz, vert, 0);
            }
        } else {
            dt.teleOpDrive(horz, vert, rotate);
        }
        dt.ftcDashDrawPose();
    }
}
