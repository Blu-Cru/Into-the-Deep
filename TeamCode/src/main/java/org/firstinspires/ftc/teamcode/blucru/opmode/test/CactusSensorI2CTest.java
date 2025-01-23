package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "test")
public class CactusSensorI2CTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "cactus");

        waitForStart();
        while (opModeIsActive()) {
            // read all 3 color channels in one I2C transmission:
            NormalizedRGBA colors = sensor.getNormalizedColors();
            double mm = sensor.getDistance(DistanceUnit.MM);
            telemetry.addData("rgb", colors.red + " " + colors.blue + " " + colors.green);
            telemetry.addData("distance (mm)", mm);
            telemetry.update();
        }
    }
}
