package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class HangServosTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addDrivetrain();
        addHangServos();
    }

    @Override
    public void periodic() {
        dt.teleOpDrive(gamepad1);

        if(stickyG1.right_stick_button) {
            dt.setHeading(Math.PI/2);
        }

        if(stickyG1.a) {
            hangServos.retract();
        } else if(stickyG1.b) {
            hangServos.release();
        } else if(stickyG1.x) {
            hangServos.hang();
        }
    }
}
