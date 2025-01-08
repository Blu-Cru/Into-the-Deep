package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class PivotExtensionManualTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addPivot();
        addExtension();
        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void periodic() {
        extension.setManualPower(-gamepad1.right_stick_y);
        pivot.setManualPower(-gamepad1.left_stick_y);
    }
}
