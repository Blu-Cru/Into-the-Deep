package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Rev Color Test", group = "test")
public class RevColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "color");

        waitForStart();

        color.initialize();

        while(opModeIsActive()) {
            float[] hsv = new float[3];
            Color.colorToHSV(color.getNormalizedColors().toColor(), hsv);

            telemetry.addData("hue", hsv[0]);
            telemetry.addData("saturation", hsv[1]);
            telemetry.addData("value", hsv[2]);
            telemetry.update();
        }
    }
}
