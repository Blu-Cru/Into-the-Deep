package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist;

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
