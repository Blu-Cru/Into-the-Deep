package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristHorizontalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class EndEffectorRetractCommand extends SequentialCommandGroup {
    public EndEffectorRetractCommand() {
        super(
                new ClampGrabCommand(),
                new ArmRetractCommand(),
                new WheelStopCommand(),
                new WristUprightForwardCommand()
        );
    }
}
