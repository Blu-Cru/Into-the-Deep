package org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristAngleCommand;

public class GrabCommand extends SequentialCommandGroup {
    public GrabCommand() {
        super(
                new ArmMotionProfileCommand(-0.45),
                new UpDownWristAngleCommand(-Math.PI/2 + 0.45),
                new WaitCommand(200),
                new ClawGrabCommand(),
                new WaitCommand(120),
                new ArmMotionProfileCommand(0),
                new UpDownWristAngleCommand(-Math.PI/2)
        );
    }
}
