package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.BoxtubeRetractFromTopBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontClipUnderneathCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenPreloadDepositPath extends PIDPathBuilder {
    public SpecimenPreloadDepositPath() {
        super();
        this.setPower(0.55)
                .callback(() -> {
//                    new SequentialCommandGroup(
//                            new PivotCommand(0.67),
//                            new ArmCommand(0.0),
//                            new UpDownWristAngleCommand(-0.1),
//                            new SpinWristAngleCommand(Math.PI),
//                            new PivotCommand(0.67),
//                            new ExtensionCommand(10.5)
//                    ).schedule();
                    new SpecimenFrontFlatCommand().schedule();
                })
                .addMappedPoint(7, -59, 90)
//                .waitMillis(100)
                .addMappedPoint(7, -45, 90)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(50),
                            new ArmRetractCommand(),
                            new WaitCommand(200),
                            new BoxtubeRetractCommand()
                    ).schedule();
                });
    }
}
