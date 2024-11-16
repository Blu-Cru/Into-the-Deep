package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class BoxtubeExtendCommand extends SequentialCommandGroup {
    public BoxtubeExtendCommand(double targetAngle, double extensionDistance) {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new PivotCommand(targetAngle),

                                // Wait time depends on distance pivot needs to rotate, longer turn requires longer wait time
                                // i have the wait time proportional to the angle needed to rotate
                                new WaitCommand((long) (Math.abs(Robot.getInstance().pivot.getAngle() - targetAngle) * 200.0)),
                                new ExtensionCommand(extensionDistance)
                        ),
                        new SequentialCommandGroup(
                                new ExtensionCommand(extensionDistance),

                                // Wait time depends on distance extension needs to extend, longer extension requires longer wait time
                                // wait time is proportional to the distance needed to extend
                                new WaitCommand((long) (Math.abs(Robot.getInstance().extension.getDistance()) - extensionDistance * 8.0)),
                                new PivotCommand(targetAngle)
                        ),

                        () -> extensionDistance > Robot.getInstance().extension.getDistance()
                )
        );
    }
}
