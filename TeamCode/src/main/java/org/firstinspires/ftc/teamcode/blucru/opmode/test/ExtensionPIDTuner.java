package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Extension PID Tuner", group = "test")
public class ExtensionPIDTuner extends BluLinearOpMode {
    public static double targetInches;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addExtension();
        addPivot();
        addArm();
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void periodic() {
        extension.updatePID();

        if(!(gamepad1.right_trigger > 0.2)) {
            extension.idle();
        } else if(gamepad1.a) {
            extension.pidTo(targetInches);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Target Inches", targetInches);
    }
}
