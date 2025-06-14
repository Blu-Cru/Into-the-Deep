package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm;

public class ArmSampleScorePositionCommand extends ArmMotionProfileCommand{
    public ArmSampleScorePositionCommand() {
        super(1.1);
    }

    public ArmSampleScorePositionCommand(boolean high) {
        super(high ? 1.1 : 1.6);
    }
}
