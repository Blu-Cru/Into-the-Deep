package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Extension Motion Profile Test", group = "test")
public class ExtensionMotionProfileTest extends BluLinearOpMode {
    public static double vMax = 25, aMax = 25;

    @Override
    public void initialize() {
        addExtension();
        addArm();
    }

    @Override
    public void periodic() {
        if(stickyG1.b) {
            extension.motionProfileTo(8, vMax, aMax);
        }

        if (stickyG1.a) {
            extension.motionProfileTo(0, 15, 20);
        }

        if(stickyG1.x) {
            extension.motionProfileTo(15, vMax, aMax);
        }
    }

    @Override
    public void telemetry() {

    }
}
