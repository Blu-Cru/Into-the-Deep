package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.SmoothServo;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.StickyGamepad;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Disabled
@Config
@TeleOp(name = "Smooth servo test", group = "test")
public class SmoothServoTest extends LinearOpMode {
    public static String name = "wrist";
    public static boolean reversed = false;
    public static double vMax = 0.1, aMax = 0.2;
    public static double position = 0.5;

    SmoothServo servo;
    StickyGamepad g1;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Globals.tele = telemetry;

        servo = new SmoothServo(name, reversed, vMax, aMax);
        g1 = new StickyGamepad(gamepad1);

        waitForStart();

        while(opModeIsActive()) {
            servo.read();
            servo.setConstraints(vMax, aMax);

            if(g1.a) {
                servo.setPosition(position);
            }

            g1.update();
            servo.write();
            servo.telemetry();
            telemetry.update();
        }
    }
}
