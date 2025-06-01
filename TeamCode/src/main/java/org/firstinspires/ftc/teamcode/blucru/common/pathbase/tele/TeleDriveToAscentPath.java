package org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TeleDriveToAscentPath extends PIDPathBuilder {
    public TeleDriveToAscentPath() {
        super();
        this.callback(() -> {
                    if(Robot.getInstance().getBoxtubePose().getY() > 25) {
                        Robot.getInstance().dt.setDrivePower(0.7);
                    } else {
                        Robot.getInstance().dt.setDrivePower(1.0);
                    }
                })
                .schedule(new SequentialCommandGroup(
                        new ArmGlobalAngleCommand(1.5),
                        new WaitCommand(150),
                        new BoxtubeRetractCommand(),
                        new EndEffectorRetractCommand()
                ))
                .addMappedPoint(-48, -30, 45, 10)
                .setPower(0.85)
                .addMappedPoint(-40, -12, 0, 8)
//                .schedule(new ArmPreIntakeCommand())
                .addMappedPoint(-26, -12, 0, 5);
    }
}
