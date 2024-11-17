package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Intake switch test", group = "test")
public class IntakeSwitchTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addIntakeSwitch();
    }
}
