package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "End effector Test", group = "test")
public class EndEffectorTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addArm();
        addWrist();
        addClamp();
        addWheel();
    }

    @Override
    public void periodic() {
        if(stickyG1.left_bumper) {
            startIntaking();
        }

        if(stickyG1.right_bumper) {
            spitOut();
        }

        if(stickyG1.a) {
            stopIntaking();
        }

        if(stickyG1.b) {
            retract();
        }
    }

    public void startIntaking() {
        arm.dropToGround();
        wrist.uprightForward();
        clamp.release();
        wheel.intake();
    }

    public void stopIntaking() {
        arm.dropToGround();
        wrist.uprightForward();
        clamp.grab();
        wheel.stop();
    }

    public void spitOut() {
        arm.dropToGround();
        wrist.uprightForward();
        clamp.release();
        wheel.reverse();
    }

    public void retract() {
        arm.retract();
        wrist.horizontal();
        clamp.grab();
        wheel.stop();
    }
}
