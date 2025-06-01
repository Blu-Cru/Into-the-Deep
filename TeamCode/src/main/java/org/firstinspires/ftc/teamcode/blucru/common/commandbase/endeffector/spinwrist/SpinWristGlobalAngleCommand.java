package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class SpinWristGlobalAngleCommand extends InstantCommand {
    public SpinWristGlobalAngleCommand(double globalAngle) {
        super(
                () -> {
                    Robot.getInstance().spinWrist.setTurretGlobalAngle(globalAngle);
                }
        );

        addRequirements(Robot.getInstance().spinWrist);
    }
}
