package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenIntakePath extends PIDPathBuilder {
    public SpecimenIntakePath(double tolerance) {
        super();
        this.setPower(0.9)
                .callback(() -> {
                    new SpecimenIntakeBackFlatCommand().schedule();
                })
                .addMappedPoint(32, -54, 110, tolerance)
                .setPower(0.25)
                .addMappedPoint(36, -63, 90)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ExtensionCommand(2.6),
                            new ClawGrabCommand(),
                            new WaitCommand(140),
                            new ExtensionCommand(7.0)
                    ).schedule();
                })
                .waitMillis(150);
    }

    public SpecimenIntakePath() {
        this(5);
    }
}
