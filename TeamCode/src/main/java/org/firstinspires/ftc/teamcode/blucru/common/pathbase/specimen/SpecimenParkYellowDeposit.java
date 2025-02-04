package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
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
                .addMappedPoint(-30, -59, 0, 16)
                .setPower(1)
                .addMappedPoint(12, -59, -10, 10)
                .callback(() -> {
                    new ExtensionCommand(15).schedule();
                    new ArmPreIntakeCommand().schedule();
                });
    }
}
