package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Pivot PID Tuner", group = "test")
public class PivotPIDTuner extends BluLinearOpMode {
    public static double targetAngle = 0.0;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addPivot();
        addExtension();
        pivot.useExtension(extension.getMotor());
    }

    @Override
    public void periodic() {
        pivot.updatePID();

        if(!(gamepad1.right_trigger > 0.2)) {
            pivot.idle();
        } else if(gamepad1.a) {
            pivot.pidTo(targetAngle);
        } else if(stickyG1.b) {
//            pivot.setMotionProfileTargetAngle(targetAngle);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Target Angle", targetAngle);
    }
}
