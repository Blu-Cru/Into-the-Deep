package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;

public class SpecimenFrontDunkRetractCommand extends SequentialCommandGroup {
    public SpecimenFrontDunkRetractCommand() {
        super(
                new ClawOpenCommand(),
                new WaitCommand(170),
                new EndEffectorRetractCommand(),
                new BoxtubeRetractCommand()
        );
    }
}
