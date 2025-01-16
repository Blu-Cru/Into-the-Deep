package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class HangMotorTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addHangMotor();
    }

    @Override
    public void periodic() {
        if(Math.abs(gamepad1.left_stick_y) > 0.1) {
            hangMotor.setManualPower(-gamepad1.left_stick_y);
        }
    }
}
