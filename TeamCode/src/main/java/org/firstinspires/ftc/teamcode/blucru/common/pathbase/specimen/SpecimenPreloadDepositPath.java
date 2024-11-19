package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.SpecimenFrontCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.SpecimenFrontDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenPreloadDepositPath extends PIDPathBuilder {
    public SpecimenPreloadDepositPath() {
        super();
        this.setPower(0.7)
                .addMappedPoint(10, -40, 90, 6)
                .schedule(new SpecimenFrontCommand())
                .setPower(0.3)
                .addMappedPoint(11, -36, 90)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(1000),
                        new SpecimenFrontDunkCommand(),
                        new WaitCommand(300),
                        new ClampReleaseCommand(),
                        new WheelReverseCommand(),
                        new WaitCommand(300),
                        new EndEffectorRetractCommand(),
                        new BoxtubeRetractCommand()
                ))
                .waitMillis(1300);
    }
}
