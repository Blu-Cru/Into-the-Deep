package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.clap_servo;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

public class ClapServos implements BluSubsystem, Subsystem {
    ClapServo[] clapServos;

    public ClapServos() {
        clapServos = new ClapServo[2];
        clapServos[0] = new RightClapServo();
        clapServos[1] = new LeftClapServo();
    }

    @Override
    public void init() {
        for (ClapServo clapServo : clapServos) {
            clapServo.init();
            clapServo.retract();
        }
    }

    @Override
    public void read() {
        for (ClapServo clapServo : clapServos) {
            clapServo.read();
        }
    }

    @Override
    public void write() {
        for (ClapServo clapServo : clapServos) {
            clapServo.write();
        }
    }

    public void retract() {
        for (ClapServo clapServo : clapServos) {
            clapServo.retract();
        }
    }

    public void center() {
        for (ClapServo clapServo : clapServos) {
            clapServo.center();
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        for (ClapServo clapServo : clapServos) {
            clapServo.telemetry();
        }
    }
}
