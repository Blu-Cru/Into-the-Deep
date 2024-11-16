package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

public class Arm extends BluServo implements BluSubsystem, Subsystem {
    public static double
            PARALLEL_POS = 0.6,
    // 90 degrees is from 0.17 to 0.45
            MAX_POS = 0.95, MIN_POS = 0.25,
            TICKS_PER_RAD = 0.1782;

    enum State{
        SERVO,
        IVK
    }

    State state;
    double globalAngle;

    public Arm() {
        super("arm");
        state = State.SERVO;
    }

    @Override
    public void init() {
        super.init();
        globalAngle = Math.PI/2;
        retract();
    }

    @Override
    public void write() {
        if(state == State.IVK) {
            super.setPosition(Range.clip(getTicksFromGlobalAngle(globalAngle), MIN_POS, MAX_POS));
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

    public void preIntake() {
        setPosition(PARALLEL_POS);
    }

    public void retract() {
        setPosition(PARALLEL_POS + 0.3);
    }

    public void dropToGround() {
        setPosition(PARALLEL_POS -0.09);
    }

    public void turnToBucket() {
        setPosition(PARALLEL_POS + 0.3);
    }

    public void highSample() {
        setPosition(PARALLEL_POS + 0.15);
    }

    private double toTicks(double rad) {
        return rad * TICKS_PER_RAD;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
