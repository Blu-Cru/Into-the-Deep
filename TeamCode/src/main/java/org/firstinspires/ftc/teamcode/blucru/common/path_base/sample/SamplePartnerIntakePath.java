package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SamplePartnerIntakePath extends PIDPathBuilder {
    public SamplePartnerIntakePath() {
        super();
        this.setPower(0.2)
                .addMappedPoint(-40, -60, 45, 4)
                .setPower(0.5)
                .addMappedPoint(-10, -60, 0, 3)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(4),
                        new WaitCommand(300),
                        new ClawOpenCommand(),
                        new WaitCommand(100),
                        new ExtensionMotionProfileCommand(14)
                ))
                .waitMillis(2000);
    }
}
