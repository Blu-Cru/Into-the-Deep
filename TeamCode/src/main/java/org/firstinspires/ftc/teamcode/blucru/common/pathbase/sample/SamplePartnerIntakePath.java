package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SamplePartnerIntakePath extends PIDPathBuilder {
    public SamplePartnerIntakePath() {
        super();
        this.setPower(0.2)
                .addMappedPoint(40, 60, 225, 4)
                .setPower(0.5)
                .addMappedPoint(10, 60, 180, 3)
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
