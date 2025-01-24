package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.LimitSwitch;

public class IntakeSwitch extends LimitSwitch {
    public IntakeSwitch() {
        super("intake switch");
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake limit switch", pressed());
    }
}
