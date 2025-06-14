package org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ExtensionMotionProfileCommand extends InstantCommand {
    public ExtensionMotionProfileCommand(double targetIn, double vMax, double aMax) {
        super(
                () -> {
                    Robot.getInstance().extension.motionProfileTo(targetIn, vMax, aMax);
                }
        );
    }

    public ExtensionMotionProfileCommand(double targetIn) {
        this(targetIn, 20, 80);
    }
}
