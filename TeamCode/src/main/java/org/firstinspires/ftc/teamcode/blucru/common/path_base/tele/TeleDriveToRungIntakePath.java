package org.firstinspires.ftc.teamcode.blucru.common.path_base.tele;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TeleDriveToRungIntakePath extends PIDPathBuilder {
    public TeleDriveToRungIntakePath() {
        super();
        this.callback(() -> {
                    if(Robot.getInstance().getBoxtubePose().getY() > 20) {
                        Robot.getInstance().dt.setDrivePower(0.8);
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
                .addMappedPoint(-5, -40, 60, 15)
//                .schedule(new ArmPreIntakeCommand())
                .setPower(0.85)
                .addMappedPoint(-5, -34, 90, 6);
    }
}
