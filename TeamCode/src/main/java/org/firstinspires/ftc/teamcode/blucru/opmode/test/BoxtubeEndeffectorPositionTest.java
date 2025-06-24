package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "test")
public class BoxtubeEndeffectorPositionTest extends BluLinearOpMode {
    public static double
            pivotAngle = 0.0,
            extensionInches = 0.0,
            turretAngle = 0.0,
            armAngle = 0.0,
            spinWristAngle = 0.0,
            upDownWristAngle = 0.0;
    @Override
    public void initialize() {
        addPivot();
        addExtension();
        addArm();
        addTurret();
        addSpinWrist();
        addUpDownWrist();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void periodic() {
        if(gamepad1.a) {
            pivot.pidTo(pivotAngle);
            extension.pidTo(extensionInches);
            arm.enable();
            arm.setAngle(armAngle);
            turret.enable();
            turret.setAngle(turretAngle);
            spinWrist.enable();
            spinWrist.setAngle(spinWristAngle);
            upDownWrist.enable();
            upDownWrist.setAngle(upDownWristAngle);
        } else {
            pivot.idle();
            extension.idle();
            arm.disable();
            turret.disable();
            spinWrist.disable();
            upDownWrist.disable();
        }
    }
}
