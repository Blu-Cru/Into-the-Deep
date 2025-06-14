package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenParkIntakePath extends PIDPathBuilder {
    public SpecimenParkIntakePath() {
        super();
        this.setPower(0.4)
//                .schedule(new BoxtubeRetractCommand())
                .addMappedPoint(24, -48, -45, 6)
                .callback(() ->
                        new SequentialCommandGroup(
                            new BoxtubeCommand(0, 8),
                            new ClawOpenCommand(),
                            new WaitCommand(300),
                            new ExtensionMotionProfileCommand(14),
                            new WaitCommand(1000),
                            new ExtensionCommand(6),
                            new WaitCommand(300),
                            new ExtensionMotionProfileCommand(14)
                        ).schedule()
                )
                .waitMillis(4000);
    }
}
