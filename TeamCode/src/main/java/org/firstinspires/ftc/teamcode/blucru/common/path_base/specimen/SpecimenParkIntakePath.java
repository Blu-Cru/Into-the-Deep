package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenParkIntakePath extends PIDPathBuilder {
    public SpecimenParkIntakePath() {
        super();
        this.setPower(0.4)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(150),
                            new FullRetractCommand()
                    ).schedule();
                })
//                .schedule(new BoxtubeRetractCommand())
                .addMappedPoint(24, -48, -45, 6)
                .callback(() ->
                        new SequentialCommandGroup(
                                new PreIntakeCommand(),
                                new WaitCommand(150),
                                new ExtensionCommand(10),
                                new WaitCommand(400),
                                new GrabCommand()
                        ).schedule()
                )
                .waitMillis(4000);
    }
}
