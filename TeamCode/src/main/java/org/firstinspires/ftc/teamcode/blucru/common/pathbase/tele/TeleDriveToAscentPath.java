package org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TeleDriveToAscentPath extends PIDPathBuilder {
    public TeleDriveToAscentPath() {
        super();
        this.callback(() -> {
                    if(Robot.getInstance().getBoxtubePose().getY() > 25) {
                        Robot.getInstance().dt.setDrivePower(0.3);
                    } else {
                        Robot.getInstance().dt.setDrivePower(0.8);
                    }
                })
                .schedule(new FullRetractCommand())
                .addMappedPoint(-48, -30, 45, 10)
                .setPower(0.85)
                .addMappedPoint(-40, -12, 0, 8)
                .addMappedPoint(-26, -12, 0, 5)
                .schedule(new ArmPreIntakeCommand());
    }
}
