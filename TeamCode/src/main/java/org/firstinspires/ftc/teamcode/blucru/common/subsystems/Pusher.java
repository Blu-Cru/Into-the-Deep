package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Pusher extends BluServo implements BluSubsystem, Subsystem {
    public static double RETRACT_POS = 0.965,
            vMAX = 6.0, aMAX = 3.5;

    enum State {
        SERVO,
        MOTION_PROFILE
    }
    State state;
    MotionProfile profile;

    public Pusher() {
        super("pusher");
        state = State.SERVO;
    }

    @Override
    public void init() {
        super.init();
        setPosition(RETRACT_POS);
        profile = new MotionProfile(getPosition(), getPosition(), vMAX, aMAX);
    }

    @Override
    public void read() {
        super.read();
    }

    @Override
    public void write() {
        switch(state) {
            case MOTION_PROFILE:
                setPosition(profile.getInstantTargetPosition());
                break;
            case SERVO:
                break;
        }

        super.write();
    }

    @Override
    public void telemetry(Telemetry tele) {
        tele.addData("Pusher state", state);
        super.telemetry();
    }

    public void retract() {
        state = State.MOTION_PROFILE;
        profile = new MotionProfile(RETRACT_POS, getPosition(), vMAX, aMAX).start();
    }

    public void extend() {
        state = State.MOTION_PROFILE;
        profile = new MotionProfile(RETRACT_POS - .315, getPosition(), vMAX, aMAX).start();
    }
}
