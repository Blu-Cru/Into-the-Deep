package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Raw Drive test", group = "test")
public class RawDriveTest extends BluLinearOpMode {
    double vert, horz, rotate;
    @Override
    public void initialize() {
        enableFTCDashboard();
        addDrivetrain();
        dt.fieldCentric = false;
    }

    @Override
    public void periodic() {
        vert = -gamepad1.left_stick_y;
        horz = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        Vector2d driveVector = new Vector2d(horz, vert).rotated(-Math.PI/2);

        dt.setWeightedDrivePower(new Pose2d(driveVector, rotate));

        if(gamepad1.right_stick_button) {
            dt.setPoseEstimate(new Pose2d(0,0,Math.PI/2));
        }

        dt.ftcDashDrawPose();
    }

    @Override
    public void telemetry() {
        dt.testTelemetry(telemetry);
    }
}
