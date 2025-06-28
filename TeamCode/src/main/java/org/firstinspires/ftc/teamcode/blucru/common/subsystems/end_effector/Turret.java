package org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Turret extends BluServo implements BluSubsystem, Subsystem {
    enum State{
        SERVO,
        BOXTUBE_SPLINE,
        MOTION_PROFILE
    }

    // positive is more clockwise looking down
    public static double
            CENTER_POS = 0.735,
            MIN_ANGLE = -1.4, MAX_ANGLE = Math.PI,
            MAX_VEL = 3.0, MAX_ACCEL = 4.0,

            TICKS_PER_RAD = 0.28/(Math.PI/2);

    public Turret() {
        super("turret");
    }

    MotionProfile profile;
    State state;
    BoxtubeSpline spline;
    double targetHeadingIVK;

    @Override
    public void init() {
        state = State.SERVO;
        super.init();
        setPosition(CENTER_POS);
    }

    @Override
    public void write() {
        switch (state) {
            case MOTION_PROFILE:
                setAngle(profile.getInstantTargetPosition());
                break;
            case BOXTUBE_SPLINE:
                setAngle(spline.states.wristAngle);
                break;
        }

        super.write();
    }

    public void followBoxtubeSpline(BoxtubeSpline spline) {
        state = State.BOXTUBE_SPLINE;
        this.spline = spline;
    }

    public void setAngle(double angle) {
        state = State.SERVO;
        angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        setPosition(CENTER_POS - angle * TICKS_PER_RAD);
    }

    public void setMotionProfileAngle(double angle) {
        state = State.MOTION_PROFILE;
        angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        profile = new MotionProfile(angle, getAngle(), MAX_VEL, MAX_ACCEL);
    }

    public double getAngle() {
        return (getPosition() - CENTER_POS) / TICKS_PER_RAD;
    }

    public void center() {
        setMotionProfileAngle(0);
    }

    public void front() {
        state = State.SERVO;
        setAngle(-Math.PI/2);
    }

    public void horizontal() {
        state = State.SERVO;
        setAngle(0);
    }

    public void back() {
        state = State.SERVO;
        setAngle(Math.PI/2);
    }

    public void opposite() {
        state = State.SERVO;
        setAngle(-Math.PI);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Wrist state", state);
        telemetry.addData("Wrist IVK heading", targetHeadingIVK);
        telemetry.addData("Wrist Angle", getAngle());
    }
}
