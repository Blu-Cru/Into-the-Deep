package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleIntakeRightPath extends PIDPathBuilder {
    public SampleIntakeRightPath() {
        super();
        this.setPower(0.2)
                .addMappedPoint(40, 50, 270, 6)
                .setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(400),
                        new ExtensionCommand(1),
                        new WaitCommand(150),
                        new ArmDropToGroundCommand(),
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand()
                ))
                .addMappedPoint(32, 31, -18, 3)
                .schedule(new SequentialCommandGroup(
                        new ArmDropToGroundCommand(),
                        new ExtensionCommand(1),
                        new WaitCommand(300),
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand(),
                        new WaitCommand(100),
                        new ExtensionCommand(4)
                ))
                .waitMillis(1000);
    }
}
