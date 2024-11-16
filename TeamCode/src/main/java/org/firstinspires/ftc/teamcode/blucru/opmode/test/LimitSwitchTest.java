package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;

@TeleOp(name = "Limit switch test", group = "test")
public class LimitSwitchTest extends LinearOpMode {
    DigitalChannel limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.get(DigitalChannelImpl.class, "limit switch");
//        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("limit switch state:", limitSwitch.getState());
            telemetry.update();
        }
    }
}
