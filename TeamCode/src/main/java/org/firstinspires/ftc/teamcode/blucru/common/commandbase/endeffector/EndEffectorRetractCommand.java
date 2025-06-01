package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;

public class EndEffectorRetractCommand extends SequentialCommandGroup {
    public EndEffectorRetractCommand() {
        super(
                new ClampGrabCommand(),
                new ArmRetractCommand(),
                new WristUprightForwardCommand()
        );
    }
}
