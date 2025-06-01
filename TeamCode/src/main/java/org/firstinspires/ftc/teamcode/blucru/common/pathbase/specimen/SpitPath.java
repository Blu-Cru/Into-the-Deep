package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpitPath extends PIDPathBuilder {
    public SpitPath() {
        super();
        this.setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(4)
                ))
                .addMappedPoint(38, -42, -60, 7)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(11),
//                        new ArmDropToGroundCommand(),
//                        new WaitCommand(100),
//                        new WheelReverseCommand(),
//                        new ClampReleaseCommand(),
//                        new WaitCommand(200),
//                        new ArmPreIntakeCommand(),
//                        new WheelStopCommand(),
                        new ClawGrabCommand(),
                        new ExtensionCommand(4)
                ))
                .waitMillis(150);
    }
}
