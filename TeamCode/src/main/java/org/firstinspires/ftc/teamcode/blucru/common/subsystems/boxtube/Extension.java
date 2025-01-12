package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose.BoxtubeIKPose;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

@Config
public class Extension implements BluSubsystem, Subsystem {
    public static double
            kP = 0.32, kI = 0.0, kD = 0.012 , kFAngle = 0.13, tolerance = 0.0,
            MIN_INCHES = 0.0, MAX_INCHES = 24.7, MAX_HORIZ_EXTENSION = 14.0,
            k_INPUT_EXTENSION = 4.0, // pid distance to change based on input
            MAX_EXTEND_POWER = 1.0, MAX_RETRACT_POWER = -1.0;

    enum State {
        IDLE,
        PID,
        MANUAL,
        BOXTUBE_SPLINE,
        IK,
        MOTION_PROFILE,
        RETRACTING,
        EXTEND_OVER_INTAKE,
        RESETTING
    }
    
    State state;
    ExtensionMotors extensionMotor;
    PDController pidController;
    MotionProfile profile;
    BoxtubeIKPose pose;
    BoxtubeSpline spline;

    PivotMotor pivot; // reference to pivot motor for feedforward
    double manualPower = 0;
    ElapsedTime resetTimer;
    double extendIntakeLength, extendIntakeDelta;

    public Extension() {
        extensionMotor = new ExtensionMotors();
        profile = new MotionProfile(0, 0, 0, 0);

        pidController = new PDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        state = State.IDLE;
        resetTimer = new ElapsedTime();

        pivot = null;

        extendIntakeLength = 0;
        extendIntakeDelta = 0;
    }

    public void init() {
        extensionMotor.init();

        state = State.IDLE;
        pidController.reset();
    }

    public void read() {
        extensionMotor.read();

        switch(state) {
            case RESETTING:
                if(resetTimer.seconds() > 0.3 && getDistance() < 0.3) {
                    resetEncoder();
                    pidTo(0);
                }
                break;
            case RETRACTING:
                if(Math.abs(extensionMotor.getDistance()) < 1.0 && Math.abs(extensionMotor.getDistanceVel()) < 0.3) {
                    state = State.RESETTING;
                    resetTimer.reset();
                }
                break;
            default:
                break;
        }
    }

    public void write() {
        Vector2d motorState = extensionMotor.getState();

        switch(state) {
            case IDLE:
            case MANUAL:
                setRawPower(manualPower);
                manualPower = 0;
                break;
            case IK:
            case PID:
            case RETRACTING:
                setPowerFeedForward(pidController.calculate(extensionMotor.getDistance()));
                break;
            case BOXTUBE_SPLINE:
                Vector2d targetState = limitState(spline.states.extensionState);
                setPowerFeedForward(pidController.calculate(motorState, targetState));
                break;
            case MOTION_PROFILE:
                setPowerFeedForward(pidController.calculate(motorState, profile));
                break;
            case EXTEND_OVER_INTAKE:
                double sp = Range.clip(extendIntakeDelta + extendIntakeLength, 0, MAX_HORIZ_EXTENSION);
                setPowerFeedForward(pidController.calculate(
                        motorState,
                        new Vector2d(sp, 0)
                        ));
                break;
            case RESETTING:
                setPowerFeedForward(-0.18);
                break;
        }

        extensionMotor.write();
    }

    public void pidTo(double inches) {
        state = State.PID;
        pidController.setSetPoint(Range.clip(inches, MIN_INCHES, MAX_INCHES));
    }

    public void motionProfileTo(double inches, double vMax, double aMax) {
        state = State.MOTION_PROFILE;
        profile = new MotionProfile(inches, getDistance(), vMax, aMax).start();
    }

    public void followBoxtubeSpline(BoxtubeSpline spline) {
        state = State.BOXTUBE_SPLINE;
        this.spline = spline;
    }

    public void setIKPose(BoxtubeIKPose pose) {
        state = State.IK;
        pidController.setSetPoint(Range.clip(pose.extensionLength, MIN_INCHES, MAX_INCHES));
        this.pose = pose;
    }

    public void retract() {
        pidTo(0);
        state = State.RETRACTING;
    }

    public double getDistance() {
        return extensionMotor.getDistance();
    }

    public void teleExtendIntake(double length) {
        state = State.EXTEND_OVER_INTAKE;
        length = Range.clip(length, 0, MAX_HORIZ_EXTENSION);
        extendIntakeLength = length;
    }

    public void teleExtendIntakeDelta(double input) {
        extendIntakeDelta = input * k_INPUT_EXTENSION;
    }

    public void manualExtendOverIntake(double input) {
        double unlimitedPos = input * k_INPUT_EXTENSION + extensionMotor.getDistance();
        pidTo(Range.clip(unlimitedPos, 0, MAX_HORIZ_EXTENSION));
    }

    public double getPIDError() {
        return Math.abs(getDistance() - pidController.getSetPoint());
    }

    public void setPowerFeedForward(double power) {
        double ff;

        if(pivot == null) ff = 0;
        else ff = Math.sin(pivot.getAngle()) * kFAngle;

        setRawPower(power + ff);
    }



    public void setRawPower(double power) {
        extensionMotor.setPower(power);
    }

    public void setManualPower(double power) {
        state = State.MANUAL;
        manualPower = power;
    }

    public void updatePID() {
        pidController.setPID(kP, kI, kD);
    }

    public Vector2d limitState(Vector2d state) {
        double vel = state.getY();

        if(state.getX() < MIN_INCHES || state.getX() > MAX_INCHES) {
            vel = 0;
        }

        return new Vector2d(Range.clip(state.getX(), MIN_INCHES, MAX_INCHES), vel);
    }

    public void idle() {
        state = State.IDLE;
        extensionMotor.setPower(0);
    }

    public void usePivot(PivotMotor pivot) {
        this.pivot = pivot;
        extensionMotor.setPivot(pivot);
    }

    public ExtensionMotors getMotor() {
        return extensionMotor;
    }

    public void resetEncoder() {
        extensionMotor.resetEncoder();
        pidController.reset();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Extension State", state);
        pidController.telemetry("Extension");
        extensionMotor.telemetry();
    }
}
