package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class PTOServosTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addPTOServos();
        addDrivetrain();

        dt.fieldCentric = false;
    }

    @Override
    public void periodic() {
        if(stickyG1.dpad_up) {
            ptoServos.disengage();
        }

        if(stickyG1.dpad_down) {
            ptoServos.engage();
        }

        dt.drive(new Pose2d(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
    }
}
