package org.firstinspires.ftc.teamcode.blucru.opmode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(group = "hardware test")
public class DistanceSensorTest extends LinearOpMode {
    public static String name = "distance0";
    Rev2mDistanceSensor sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, name);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Name", name);
            telemetry.addData("Inches", sensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("mm", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
