package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WheelPowerCommand extends InstantCommand {
    public WheelPowerCommand(double power) {
        super(
                () -> Robot.getInstance().wheel.setPower(power)
        );
    }
}
