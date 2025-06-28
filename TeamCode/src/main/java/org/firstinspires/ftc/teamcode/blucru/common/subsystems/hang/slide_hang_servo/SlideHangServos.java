package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.slide_hang_servo;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

import java.util.ArrayList;
import java.util.List;

public class SlideHangServos implements Subsystem, BluSubsystem {
    enum State {
        RETRACTED,
        RELEASED,
        HANGING
    }
    State state;
    List<SlideHangServo> servos;

    public SlideHangServos() {
        state = State.RETRACTED;
        servos = new ArrayList<>();
        servos.add(new LeftSlideHangServo());
        servos.add(new RightSlideHangServo());
    }

    @Override
    public void init() {
        for (SlideHangServo servo : servos) {
            servo.init();
//            servo.disable();
        }
    }

    @Override
    public void read() {
        for (SlideHangServo servo : servos) {
            servo.read();
        }
    }

    @Override
    public void write() {
        for (SlideHangServo servo : servos) {
            servo.write();
        }
    }

    public void retract() {
        state = State.RETRACTED;

        for (SlideHangServo servo : servos) {
            servo.retract();
        }
    }

    public void midway() {
        for(SlideHangServo servo : servos) {
            servo.midway();
        }
    }

    public void release() {
        state = State.RELEASED;

        for (SlideHangServo servo : servos) {
            servo.release();
        }
    }

    public void hang() {
        state = State.HANGING;

        for (SlideHangServo servo : servos) {
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
            default:
                hang();
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        for (SlideHangServo servo : servos) {
            servo.telemetry();
        }
    }
}
