package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

import java.util.ArrayList;
import java.util.List;

public class HangServos implements Subsystem, BluSubsystem {
    enum State {
        RETRACTED,
        RELEASED,
        HANGING
    }
    State state;
    List<HangServo> servos;

    public HangServos() {
        state = State.RETRACTED;
        servos = new ArrayList<>();
        servos.add(new LeftHangServo());
        servos.add(new RightHangServo());
    }

    @Override
    public void init() {
        for (HangServo servo : servos) {
            servo.init();
            servo.disable();
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
        state = State.RETRACTED;

        for (HangServo servo : servos) {
            servo.retract();
        }
    }

    public void release() {
        state = State.RELEASED;

        for (HangServo servo : servos) {
            servo.release();
        }
    }

    public void hang() {
        state = State.HANGING;

        for (HangServo servo : servos) {
            servo.hang();
        }
    }

    public void toggle() {
        switch (state) {
            case RETRACTED:
            case HANGING:
                release();
                break;
            case RELEASED:
                hang();
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        for (HangServo servo : servos) {
            servo.telemetry();
        }
    }
}
