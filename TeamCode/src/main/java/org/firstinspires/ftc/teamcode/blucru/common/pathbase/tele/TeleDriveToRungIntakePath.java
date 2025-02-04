package org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TeleDriveToRungIntakePath extends PIDPathBuilder {
    public TeleDriveToRungIntakePath() {
        super();
        this.callback(() -> {
                    if(Robot.getInstance().getBoxtubePose().getY() > 20) {
                        Robot.getInstance().dt.setDrivePower(0.3);
                    } else {
                        Robot.getInstance().dt.setDrivePower(0.8);
                    }
                })
                .schedule(new FullRetractCommand())
                .addMappedPoint(-5, -40, 60, 15)
                .setPower(0.85)
                .addMappedPoint(-5, -34, 90, 6)
                .schedule(new ArmPreIntakeCommand());
    }
}
