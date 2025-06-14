package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleLiftHighFromSubPath extends PIDPathBuilder {
    public SampleLiftHighFromSubPath() {
        super();
        this.setPower(0.8)
                .addMappedPoint(-42, -12, 20, 10)
                .schedule(new SequentialCommandGroup(
                        new PivotCommand(Math.PI/2),
                        new ArmRetractCommand()
                ))
                .addMappedPoint(-53, -53, 45, 10)
                .callback(() -> new SampleBackHighCommand().schedule())
                .addMappedPoint(-53, -53, 45);
    }
}
