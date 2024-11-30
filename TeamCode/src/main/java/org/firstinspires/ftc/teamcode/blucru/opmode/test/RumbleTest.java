package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Rumble Test", group = "test")
public class RumbleTest extends BluLinearOpMode {
    @Override
    public void periodic() {
        if(stickyG1.a || stickyG2.a) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }

        if(stickyG1.b || stickyG2.b) {
            gamepad1.rumble(1.0, 0.0, 1000);
            gamepad2.rumble(1.0, 0.0, 1000);
        }

        if(stickyG1.x || stickyG2.x) {
            gamepad1.rumble(0.0, 1.0, 1000);
            gamepad2.rumble(0.0, 1.0, 1000);
        }

        if(stickyG1.y || stickyG2.y) {
            gamepad1.rumble(0.5, 0.5, 1000);
            gamepad2.rumble(0.5, 0.5, 1000);
        }
    }
}
