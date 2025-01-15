package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

import java.util.List;

public class HangServos implements Subsystem, BluSubsystem {
    List<HangServo> servos;

    public HangServos() {
        servos.add(new LeftHangServo());
        servos.add(new RightHangServo());
    }

    @Override
    public void init() {
        for (HangServo servo : servos) {
            servo.init();
        }
    }

    @Override
    public void read() {
        for (HangServo servo : servos) {
            servo.read();
        }
    }

    @Override
    public void write() {
        for (HangServo servo : servos) {
            servo.write();
        }
    }

    public void retract() {
        for (HangServo servo : servos) {
            servo.retract();
        }
    }

    public void release() {
        for (HangServo servo : servos) {
            servo.release();
        }
    }

    public void hang() {
        for (HangServo servo : servos) {
            servo.hang();
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        for (HangServo servo : servos) {
            servo.telemetry();
        }
    }
}
