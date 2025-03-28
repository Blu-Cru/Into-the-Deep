package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Wrist extends BluServo implements BluSubsystem, Subsystem {
    enum State{
        SERVO,
        BOXTUBE_SPLINE,
        MOTION_PROFILE,
        HEADING_IVK_DOWN
    }

    public static double HORIZONTAL_POS = 0.595,
            MIN_ANGLE = -Math.PI, MAX_ANGLE = Math.PI/2,

            TICKS_PER_RAD = 0.28/(Math.PI/2);
    public Wrist() {
        super("wrist");
    }

    MotionProfile profile;
    State state;
    BoxtubeSpline spline;
    double targetHeadingIVK;

    @Override
    public void init() {
        state = State.SERVO;
        super.init();
        front();
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
            case HEADING_IVK_DOWN:
                double angle = Angle.normDelta(targetHeadingIVK - Robot.getInstance().dt.heading);
                if(angle > MAX_ANGLE) angle -= Math.PI;

                setAngle(angle);
                break;
            case SERVO:
                break;
        }

        super.write();
    }

    public void followBoxtubeSpline(BoxtubeSpline spline) {
        state = State.BOXTUBE_SPLINE;
        this.spline = spline;
    }

    public void setAngle(double angle) {
        angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        setPosition(HORIZONTAL_POS + angle * TICKS_PER_RAD);
    }

    public double getAngle() {
        return (getPosition() - HORIZONTAL_POS) / TICKS_PER_RAD;
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

    public void setHeadingIVK(double heading) {
        state = State.HEADING_IVK_DOWN;
        targetHeadingIVK = heading;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Wrist state", state);
        telemetry.addData("Wrist IVK heading", targetHeadingIVK);
        telemetry.addData("Wrist Angle", getAngle());
    }
}
