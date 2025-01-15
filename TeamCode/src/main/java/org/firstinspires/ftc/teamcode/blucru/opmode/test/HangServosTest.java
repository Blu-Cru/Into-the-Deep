package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class HangServosTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addHangServos();
    }

    @Override
    public void periodic() {
        if(stickyG1.a) {
            hangServos.retract();
        } else if(stickyG1.b) {
            hangServos.release();
        } else if(stickyG1.x) {
            hangServos.hang();
        }
    }
}
