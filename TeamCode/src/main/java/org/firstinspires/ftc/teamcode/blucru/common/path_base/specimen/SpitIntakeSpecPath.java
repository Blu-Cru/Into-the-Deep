package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpitIntakeSpecPath extends PIDPathBuilder {
    public SpitIntakeSpecPath(double x) {
        super();
        this.setPower(0.9)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new EndEffectorRetractCommand(),
                            new BoxtubeCommand(1.3, 0)
                    ).schedule();
                })
                .addMappedPoint(36, -55, 90, 12)
                .callback(() -> {
                    new SpecimenIntakeBackFlatSpitCommand().schedule();
                })
//                .callback(() -> {
//                    new SequentialCommandGroup(
//                            new UpDownWristAngleCommand(-1.0),
//                            new WaitCommand(150),
//                            new ClawOpenCommand()
//                    ).schedule();
//                })
                .waitMillis(800)
                .setPower(0.25)
                .addMappedPoint(x, -61, 90)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new WaitCommand(140),
                            new PivotCommand(1.0),
                            new UpDownWristAngleCommand(-2.0)
                    ).schedule();
                })
                .waitMillis(250);
    }

    public SpitIntakeSpecPath() {
        this(36);
    }
}
