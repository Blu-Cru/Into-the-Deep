package org.firstinspires.ftc.teamcode.blucru.opmode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Servo test", group = "hardware test")
public class ServoTest extends LinearOpMode {
    public static double position = 0.5;
    public static String name = "wrist";
    public static boolean reversed = false;
    ServoImplEx test;
    ServoControllerEx controller;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            try {
                test = hardwareMap.get(ServoImplEx.class, name);
            } catch (Exception e) {
                telemetry.addLine("ERROR: servo " + name + " not found");
                telemetry.update();
                continue;
            }

            try {
                updateDirection();
                updateController();
            } catch (Exception e) {
                telemetry.addLine("ERROR updating direction and controller for" + name);
                telemetry.update();
                continue;
            }

            if(gamepad1.a) {
                controller.pwmEnable();
                test.setPosition(position);
            } else {
                controller.pwmDisable();
            }

            telemetry.addData("name", name);
            telemetry.addData("position", test.getPosition());
            telemetry.update();
        }
    }

    public void updateName() {
        try {
            test = hardwareMap.get(ServoImplEx.class, name);
        } catch (Exception e) {
            telemetry.addLine("ERROR: servo " + name + " not found");
        }
    }

    public void updateDirection() {
        try {
            if(reversed) test.setDirection(Servo.Direction.REVERSE);
            else test.setDirection(Servo.Direction.FORWARD);
        } catch (Exception e) {}
    }

    public void updateController() {
        try{
            controller = (ServoControllerEx) test.getController();
        } catch (Exception e) {}
    }

    public void disable() {
        try {
            controller.setServoPwmDisable(test.getPortNumber());
        } catch (Exception e) {}
    }
}
