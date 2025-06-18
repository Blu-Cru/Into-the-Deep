package org.firstinspires.ftc.teamcode.blucru.opmode.test.cactus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class CactusDigitalTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addCactus();
    }
}
