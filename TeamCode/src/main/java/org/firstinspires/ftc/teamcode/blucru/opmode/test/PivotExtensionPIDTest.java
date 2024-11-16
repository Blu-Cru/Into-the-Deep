package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Pivot Extension PID Test", group = "test")
public class PivotExtensionPIDTest extends BluLinearOpMode {
    public static double targetAngle = 0, targetExtension = 0;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addPivot();
        addExtension();
        addArm();
        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void periodic() {
        extension.updatePID();
        pivot.updatePID();

        if(!(gamepad1.right_trigger > 0.2)) {
            pivot.idle();
            extension.idle();

            if(gamepad1.right_stick_button) {
                pivot.resetEncoder();
                extension.resetEncoder();
            }
        } else if(gamepad1.a) {
            pivot.pidTo(targetAngle);
            extension.pidTo(targetExtension);
        } else if(gamepad1.b) {
            pivot.pidTo(0);
            extension.pidTo(0);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Target angle", targetAngle);
        telemetry.addData("Target extension", targetExtension);
    }
}
