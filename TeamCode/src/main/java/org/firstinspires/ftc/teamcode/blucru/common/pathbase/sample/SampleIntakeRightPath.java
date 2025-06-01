package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleIntakeRightPath extends PIDPathBuilder {
    public SampleIntakeRightPath() {
        super();
        this.setPower(0.9)
                .addMappedPoint(-40, -50, 90, 4)
                .setPower(0.9)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(150),
                        new ExtensionCommand(2)
                ))
                .addMappedPoint(-32, -32, 162, 8)
                .schedule(new SequentialCommandGroup(
                        new ClawOpenCommand(),
                        new ExtensionMotionProfileCommand(8)
                ))
                .waitMillis(600);
    }
}
