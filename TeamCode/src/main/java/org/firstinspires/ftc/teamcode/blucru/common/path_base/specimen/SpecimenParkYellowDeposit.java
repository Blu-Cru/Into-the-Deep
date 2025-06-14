package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenParkYellowDeposit extends PIDPathBuilder {
    public SpecimenParkYellowDeposit() {
        super();
        this.setPower(0.7)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.5),
                            new WaitCommand(150),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .addMappedPoint(-30, -53, 0, 16)
                .setPower(1)
                .addMappedPoint(37, -57, -10, 10)
                .callback(() -> {
//                    new ExtensionCommand(15).schedule();
//                    new ArmPreIntakeCommand().schedule();
                });
    }
}
