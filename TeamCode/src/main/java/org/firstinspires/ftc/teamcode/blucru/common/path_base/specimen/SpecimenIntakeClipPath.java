package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenIntakeClipPath extends PIDPathBuilder {
    public SpecimenIntakeClipPath(double tolerance) {
        super();
        this.setPower(0.7)
                .callback(() -> {
                    new SpecimenIntakeBackClipCommand().schedule();
                })
                .addMappedPoint(32, -54, 110, tolerance)
                .setPower(0.25)
                .addMappedPoint(36, -62, 90)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new WaitCommand(140),
                            new ExtensionCommand(7.0)
                    ).schedule();
                })
                .waitMillis(150);
    }

    public SpecimenIntakeClipPath() {
        this(5);
    }
}
