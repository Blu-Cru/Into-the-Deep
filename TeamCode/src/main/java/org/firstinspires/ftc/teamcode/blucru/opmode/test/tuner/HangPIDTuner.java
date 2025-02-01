package org.firstinspires.ftc.teamcode.blucru.opmode.test.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "test")
public class HangPIDTuner extends BluLinearOpMode {
    public static double targetTicks = 0;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addHangMotor();
        addArm();
    }

    @Override
    public void periodic() {
        hangMotor.updatePID();

        if(!(gamepad1.right_trigger > 0.2)) {
            hangMotor.idle();
        } else if(gamepad1.a) {
            hangMotor.pidTo(targetTicks);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Target ticks", targetTicks);
    }
}
