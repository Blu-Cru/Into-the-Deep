package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleHighLiftPath extends PIDPathBuilder {
    public SampleHighLiftPath() {
        super();
        this.setPower(0.8)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new BoxtubeCommand(Math.PI/2, 0),
                            new EndEffectorRetractCommand(),
                            new ArmCommand(0.0),
                            new WaitCommand(700),
                            new SampleBackHighCommand()
                    ).schedule();
                })
                .addMappedPoint(-53, -53, 45);
    }
}