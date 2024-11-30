package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Disabled
@TeleOp(name = "Drive Base Test", group = "test")
public class DriveBaseTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addDrivetrain();
    }

    @Override
    public void initLoop() {
        telemetry.addLine("Initialized");
    }

    @Override
    public void periodic() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;


        Pose2d drivePose = new Pose2d(x, y, rot);

        if(gamepad1.right_stick_button) {
            dt.setHeading(Math.PI/2);
        }

//        Pose2d drivePose = new Pose2d(new Vector2d(x,y).rotated(-Math.PI/2), rot);
        dt.driveFieldCentric(drivePose);
    }
}
