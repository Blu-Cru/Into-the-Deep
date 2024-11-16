package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple.*;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

import java.util.HashMap;

@Disabled
@Config
@TeleOp(name = "Blu motor test", group = "test")
public class BluMotorTest extends LinearOpMode {
    HashMap<Boolean, Direction> directions = new HashMap<Boolean, Direction>() {{
        put(true, Direction.REVERSE);
        put(false, Direction.FORWARD);
    }};

    HashMap<Boolean, ZeroPowerBehavior> zpbs = new HashMap<Boolean, ZeroPowerBehavior>() {{
        put(true, ZeroPowerBehavior.BRAKE);
        put(false, ZeroPowerBehavior.FLOAT);
    }};

    public static String name = "intake motor";
    public static boolean reversed = true, brake = true;
    BluMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Globals.tele = telemetry;

        motor = new BluMotor(name, directions.get(reversed), zpbs.get(brake));

        waitForStart();

        while (opModeIsActive()) {
            motor.read();
            if(Math.abs(gamepad1.right_stick_y) > 0.02) {
                motor.setPower(-gamepad1.right_stick_y);
            } else motor.setPower(0);

            motor.write();
            motor.telemetry();
            telemetry.update();
        }
    }
}
