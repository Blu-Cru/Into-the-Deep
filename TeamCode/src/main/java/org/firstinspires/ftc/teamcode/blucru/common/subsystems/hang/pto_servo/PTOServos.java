package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.pto_servo;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

public class PTOServos implements BluSubsystem, Subsystem {
    LeftPTOServo leftPTO;
    RightPTOServo rightPTO;

    public PTOServos() {
        leftPTO = new LeftPTOServo();
        rightPTO = new RightPTOServo();
    }

    @Override
    public void init() {
        leftPTO.init();
        rightPTO.init();
    }

    @Override
    public void read() {
        leftPTO.read();
        rightPTO.read();
    }

    @Override
    public void write() {
        leftPTO.write();
        rightPTO.write();
    }

    public void engage() {
        leftPTO.engage();
        rightPTO.engage();
    }

    public void disengage() {
        leftPTO.disengage();
        rightPTO.disengage();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        leftPTO.telemetry();
        rightPTO.telemetry();
    }
}
