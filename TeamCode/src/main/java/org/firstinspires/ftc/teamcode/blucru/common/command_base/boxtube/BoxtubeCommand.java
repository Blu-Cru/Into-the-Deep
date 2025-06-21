package org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class BoxtubeCommand extends SequentialCommandGroup {
    public BoxtubeCommand(double targetAngle, double targetExtension) {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new PivotCommand(targetAngle),

                                // Wait time depends on distance pivot needs to rotate, longer turn requires longer wait time
                                // wait for extension proportional to the angle needed to rotate
                                new WaitCommand((long) (Math.abs(Robot.getInstance().pivot.getAngle() - targetAngle) * 250.0)),
                                new ExtensionCommand(targetExtension)
                        ),
                        new SequentialCommandGroup(
                                new ExtensionCommand(targetExtension),

                                // Wait time depends on distance extension needs to extend, longer extension requires longer wait time
                                // wait for pivot proportional to the distance needed to extend
                                new WaitCommand((long) (Math.abs(Robot.getInstance().extension.getDistance() - targetExtension) * 5.0)),
                                new PivotCommand(targetAngle)
                        ),

                        () -> targetExtension > Robot.getInstance().extension.getDistance()
                )
        );
    }
}
