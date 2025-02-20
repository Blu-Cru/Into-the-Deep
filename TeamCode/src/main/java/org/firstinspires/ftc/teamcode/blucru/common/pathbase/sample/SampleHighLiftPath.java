package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleHighLiftPath extends PIDPathBuilder {
    public SampleHighLiftPath() {
        super();
        this.setPower(0.8)
                .schedule(new SequentialCommandGroup(
                        new PivotCommand(Math.PI/2),
                        new ArmRetractCommand()
                ))
                .addMappedPoint(-53, -53, 45, 10)
                .callback(() -> new SampleBackHighCommand().schedule())
                .addMappedPoint(-53, -53, 45);
    }
}
