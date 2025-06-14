package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;

public class SpecimenBackDunkRetractCommand extends SequentialCommandGroup {
    public SpecimenBackDunkRetractCommand() {
        super(
                new ClawOpenCommand(),
                new WaitCommand(300),
                new BoxtubeRetractCommand(),
                new EndEffectorRetractCommand()
        );
    }
}
