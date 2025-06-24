package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleIntakeLeftPath extends PIDPathBuilder {
    public SampleIntakeLeftPath() {
        super();
        this.setPower(0.7)
                .addMappedPoint(-62, -46, 120, 4)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(150),
                        new PivotRetractCommand(),
                        new WaitCommand(300),
                        new ExtensionCommand(2)
                ))
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(70),
                        new ClawOpenCommand(),
                        new ExtensionMotionProfileCommand(8)
                ))
                .waitMillis(4000);
    }
}
