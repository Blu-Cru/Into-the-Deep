package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Extension implements BluSubsystem, Subsystem {
    public static double
            kP = 0.8, kI = 0.0, kD = 0.01, kFAngle = 0.1, tolerance = 0.0,
            MIN_INCHES = 0.0, MAX_INCHES = 24,
            MAX_EXTEND_POWER = 1.0, MAX_RETRACT_POWER = -1.0;

    enum State {
        IDLE,
        PID,
        MANUAL,
        MOTION_PROFILE,
        RETRACTING,
        RESETTING
    }
    
    State state;
    ExtensionMotor extensionMotor;
    PIDController pidController;
    MotionProfile profile;
    int retractionCount;

    PivotMotor pivot; // reference to pivot motor for feedforward
    double manualPower = 0;
    ElapsedTime resetTimer;

    public Extension() {
        extensionMotor = new ExtensionMotor();
        profile = new MotionProfile(0, 0, 0, 0);

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        state = State.IDLE;
        resetTimer = new ElapsedTime();

        pivot = null;

        retractionCount = 0;
    }

    public void init() {
        extensionMotor.init();

        state = State.IDLE;
        pidController.reset();
    }

    public void read() {
        extensionMotor.read();

        switch(state) {
            case IDLE:
            case MANUAL:
                break;
            case PID:
            case MOTION_PROFILE:
                break;
            case RETRACTING:
                if(Math.abs(extensionMotor.getDistance()) < 1.0 && Math.abs(extensionMotor.getDistanceVel()) < 0.3) {
                    state = State.RESETTING;
                    resetTimer.reset();
                }
                break;
            case RESETTING:
                if(resetTimer.seconds() > 0.3 && getDistance() < 0.3) {
                    extensionMotor.resetEncoder();
                    pidTo(0);
                }
                break;
        }
    }

    public void write() {
        switch(state) {
            case IDLE:
            case MANUAL:
                setPowerFeedForward(manualPower);
                manualPower = 0;
                break;
            case PID:
            case RETRACTING:
                setPowerFeedForward(pidController.calculate(extensionMotor.getDistance()));
                break;
            case RESETTING:
                setPowerFeedForward(-0.15);
                break;
        }

        extensionMotor.write();
    }

    public void pidTo(double inches) {
        state = State.PID;
        pidController.setSetPoint(Range.clip(inches, MIN_INCHES, MAX_INCHES));
    }

    public void retract() {
        pidTo(0);
        state = State.RETRACTING;
    }

    public double getDistance() {
        return extensionMotor.getDistance();
    }

    public double getFeedForward(double pivotAngle) {
        return Math.sin(pivotAngle) * kFAngle;
    }

    public void setManualIntakingPower(double power) {
        state = State.MANUAL;
        manualPower = power;
    }

    private void setPowerFeedForward(double power) {
        double ff;

        if(pivot == null) ff = 0;
        else ff = getFeedForward(pivot.getAngle());

        extensionMotor.setPower(Range.clip(power + ff, MAX_RETRACT_POWER, MAX_EXTEND_POWER));
    }

    public void updatePID() {
        pidController.setPID(kP, kI, kD);
    }

    public void idle() {
        state = State.IDLE;
        extensionMotor.setPower(0);
    }

    public void usePivot(PivotMotor pivot) {
        this.pivot = pivot;
    }

    public ExtensionMotor getMotor() {
        return extensionMotor;
    }

    public void resetEncoder() {
        extensionMotor.resetEncoder();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Extension State", state);
        extensionMotor.telemetry();
    }
}
