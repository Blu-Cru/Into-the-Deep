package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class BoxtubeRetractCommand extends SequentialCommandGroup {
    public BoxtubeRetractCommand() {
        super(
                new SequentialCommandGroup(
                        new ExtensionRetractCommand(),

                        // Wait time depends on distance extension needs to extend. Longer extension requires longer wait time.
                        // here i do distance in inches times 15, so at 15 inches it will wait 225 ms
                        new WaitCommand((long) (Robot.getInstance().extension.getDistance() * 8.0)),
                        new PivotRetractCommand()
                )
        );
    }
}
