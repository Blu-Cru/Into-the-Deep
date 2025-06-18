package org.firstinspires.ftc.teamcode.blucru.opmode.test.cactus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(group = "test")
public class CactusAnalogTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput pin0 = hardwareMap.get(AnalogInput.class, "analog");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analog hue", pin0.getVoltage() / 3.3 * 360.0);
            telemetry.update();
        }
    }
}
