package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDisableCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleParkPath extends PIDPathBuilder {
    public SampleParkPath() {
        super();
        this.setPower(0.6)
                .addMappedPoint(-50, -40, 90, 4)
                .setPower(0.8)
                .addMappedPoint(-48, -20, 135, 8)
                .addMappedPoint(-26, -12, 180, 4)
                .schedule(new SequentialCommandGroup(
                        new ArmGlobalAngleCommand(Math.PI/2),
                        new PivotCommand(1.65),
                        new WaitCommand(500),
                        new ArmDisableCommand())
                )
                .addMappedPoint(-23.5, -12, 180);
    }
}
