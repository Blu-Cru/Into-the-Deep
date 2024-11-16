package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.ExtensionMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.PivotMotor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@TeleOp(name = "Pivot Extension Encoder Test", group = "test")
public class PivotExtensionEncoderTest extends LinearOpMode {
    ExtensionMotor extension;
    PivotMotor pivot;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;

        extension = new ExtensionMotor();
        pivot = new PivotMotor();
        extension.init();
        pivot.init();

        waitForStart();

        while (opModeIsActive()) {
            extension.read();
            pivot.read();

            extension.telemetry();
            pivot.telemetry();
            telemetry.update();
        }
    }
}
