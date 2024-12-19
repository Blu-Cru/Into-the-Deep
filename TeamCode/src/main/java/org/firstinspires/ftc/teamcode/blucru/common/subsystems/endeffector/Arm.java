package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose.BoxtubeIKPose;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Arm extends BluServo implements BluSubsystem, Subsystem {
    public static double
            PARALLEL_POS = 0.45,
            vMAX = 25.0, aMAX = 30.0,
            MAX_ANGLE = 1.8, MIN_ANGLE = -1.7,
            RETRACT_ANGLE = 1.7,
            PRE_INTAKE_ANGLE = 0.15,
            GROUND_ANGLE = -0.58,

            TICKS_PER_RAD = 0.1782;

    enum State{
        SERVO,
        PIVOT_IK,
        BOXTUBE_SPLINE,
        MOTION_PROFILE
    }

    State state;
    double globalAngle;
    MotionProfile profile;
    BoxtubeSpline spline;

    public Arm() {
        super("arm");
        state = State.SERVO;
    }

    @Override
    public void init() {
        super.init();
        setAngle(RETRACT_ANGLE);
        profile = new MotionProfile(getPosition(), getPosition(), vMAX, aMAX);
    }

    @Override
    public void write() {
        switch(state) {
            case PIVOT_IK:
                setAngle(globalAngle - Robot.getInstance().pivot.getAngle());
                break;
            case MOTION_PROFILE:
                setAngle(profile.getInstantTargetPosition());
                break;
            case BOXTUBE_SPLINE:
                setAngle(spline.states.armAngle);
                break;
            case SERVO:
                break;
        }

        super.write();
    }

    public void setGlobalAngle(double globalAngle) {
        state = State.PIVOT_IK;
        this.globalAngle = globalAngle;
    }

    public void followBoxtubeSpline(BoxtubeSpline spline) {
        state = State.BOXTUBE_SPLINE;
        this.spline = spline;
    }

    public void setIKPose(BoxtubeIKPose pose) {
        state = State.SERVO;
        setAngle(pose.armAngle);
    }

    public void setAngle(double angle) {
        super.setPosition(PARALLEL_POS + toTicks(Range.clip(angle, MIN_ANGLE, MAX_ANGLE)));
    }

    public double getAngle() {
        return toRad(super.getPosition() - PARALLEL_POS);
    }

    public void setMotionProfileAngle(double targetRad) {
        state = State.MOTION_PROFILE;
        profile = new MotionProfile(targetRad, getAngle(), vMAX, aMAX).start();
    }

    public void preIntake() {
        setMotionProfileAngle(PRE_INTAKE_ANGLE);
    }

    public void retract() {
        setMotionProfileAngle(RETRACT_ANGLE);
    }

    public void dropToGround() {
        setMotionProfileAngle(GROUND_ANGLE);
    }

    private double toTicks(double rad) {
        return rad * TICKS_PER_RAD;
    }

    private double toRad(double ticks) {
        return ticks / TICKS_PER_RAD;
    }

    public void disable() {
        state = State.SERVO;
        super.disable();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Arm Angle", getAngle());
    }
}
