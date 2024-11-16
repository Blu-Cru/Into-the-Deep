package org.firstinspires.ftc.teamcode.blucru.opmode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Motor test", group = "hardware test")
public class MotorTest extends LinearOpMode {
    public static String name = "hang";
    public static boolean reversed = false;
    DcMotorEx test;

    @Override
    public void runOpMode() throws InterruptedException {
        updateName();
        updateDirection();
        reset();

        waitForStart();
        while(opModeIsActive()) {
            updateName();
            updateDirection();

            double input = -gamepad1.left_stick_y;

            if(Math.abs(input) > 0.1) test.setPower(input);
            else test.setPower(0);

            if(gamepad1.left_stick_button) reset();

            telemetry.addData("name", name);
            telemetry.addData("power", test.getPower());
            telemetry.addData("current position", test.getCurrentPosition());
            telemetry.update();
        }
    }

    public void updateDirection() {
        try {

            if (reversed) test.setDirection(DcMotorSimple.Direction.REVERSE);
            else test.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch(Exception ignored) {}
    }

    public void updateName() {
        try {
            test = hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            telemetry.addLine("ERROR: motor " + name + " not found");
        }
    }

    public void reset() {
        try {
            test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            test.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception ignored) {}
    }
}
