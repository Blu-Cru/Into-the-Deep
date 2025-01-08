package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class PusherTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addPusher();
    }

    @Override
    public void periodic() {
        if(gamepad1.a) {
            pusher.retract();
        }

        if(gamepad1.b) {
            pusher.extend();
        }
    }
}
