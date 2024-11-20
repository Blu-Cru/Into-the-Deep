package org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;

public class SpecimenFrontDunkRetractCommand extends SequentialCommandGroup {
    public SpecimenFrontDunkRetractCommand() {
        super(
                new ClampReleaseCommand(),
                new WheelReverseCommand(),
                new WaitCommand(300),
                new EndEffectorRetractCommand(),
                new BoxtubeRetractCommand()
        );
    }
}
