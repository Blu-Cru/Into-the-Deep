package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Arm extends BluServo implements BluSubsystem, Subsystem {
    public static double
            PARALLEL_POS = 0.45,
            vMAX = 6.0, aMAX = 7.0,
    // 90 degrees is from 0.17 to 0.45
            MAX_POS = PARALLEL_POS + 0.32, MIN_POS = PARALLEL_POS - 0.32,
            RETRACT_POS = PARALLEL_POS + 0.3,
            PRE_INTAKE_POS = PARALLEL_POS + 0.05,
            GROUND_POS = PARALLEL_POS -0.105,

            TICKS_PER_RAD = 0.1782;

    enum State{
        SERVO,
        IVK,
        MOTION_PROFILE
    }

    State state;
    double globalAngle;
    MotionProfile profile;

    public Arm() {
        super("arm");
        state = State.SERVO;
    }

    @Override
    public void init() {
        super.init();
        globalAngle = Math.PI/2;
        preIntake();
        profile = new MotionProfile(getPosition(), getPosition(), vMAX, aMAX);
    }

    @Override
    public void write() {
        switch(state) {
            case IVK:
                super.setPosition(Range.clip(getTicksFromGlobalAngle(globalAngle), MIN_POS, MAX_POS));
                break;
            case MOTION_PROFILE:
                super.setPosition(profile.getInstantTargetPosition());
                break;
            case SERVO:
                break;
        }

        super.write();
    }

    public void setGlobalAngle(double globalAngle) {
        state = State.IVK;
        this.globalAngle = globalAngle;
    }

    public double getTicksFromGlobalAngle(double globalAngle) {
        return toTicks(globalAngle - Robot.getInstance().pivot.getAngle()) + PARALLEL_POS;
    }

    public void setPosition(double position) {
        state = State.SERVO;
        super.setPosition(Range.clip(position, MIN_POS, MAX_POS));
    }

    public void setMotionProfilePosition(double targetPos) {
        state = State.MOTION_PROFILE;
        profile = new MotionProfile(targetPos, getPosition(), vMAX, aMAX).start();
    }

    public void preIntake() {
        setMotionProfilePosition(PRE_INTAKE_POS);
    }

    public void retract() {
        setMotionProfilePosition(RETRACT_POS);
    }

    public void dropToGround() {
        setMotionProfilePosition(GROUND_POS);
    }

    private double toTicks(double rad) {
        return rad * TICKS_PER_RAD;
    }

    public void disable() {
        state = State.SERVO;
        super.disable();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
