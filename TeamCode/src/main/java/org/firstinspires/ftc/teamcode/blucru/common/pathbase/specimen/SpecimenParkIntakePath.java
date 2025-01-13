package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenParkIntakePath extends PIDPathBuilder {
    public SpecimenParkIntakePath() {
        super();
        this.setPower(0.4)
//                .schedule(new BoxtubeRetractCommand())
                .addMappedPoint(24, -48, -45, 6)
                .schedule(new SequentialCommandGroup(
                        new BoxtubeCommand(0, 8),
                        new ArmDropToGroundCommand(),
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand(),
                        new WaitCommand(300),
                        new ExtensionMotionProfileCommand(14),
                        new WaitCommand(1000),
                        new ExtensionCommand(6),
                        new WaitCommand(300),
                        new ExtensionMotionProfileCommand(14)
                ))
                .waitMillis(5000);
    }
}
