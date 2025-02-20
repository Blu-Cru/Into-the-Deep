package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose.BoxtubeIKPose;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

@Config
public class Pivot implements BluSubsystem, Subsystem {
    public static double
            kP = 5.5, kI = 0.0, kD = 0.28, tolerance = 0.0,
            kFF_COS = 0.11, kFF_EXTENSION = 0.011,
            MIN_RAD = 0.0, MAX_RAD = 1.9,
            MAX_UP_POWER = 1.0, MAX_DOWN_POWER = -0.75,
            MAX_VELO = 1.0, MAX_ACCEL = 0.5;

    enum State {
        IDLE,
        MANUAL,
        IK,
        BOXTUBE_SPLINE,
        PID,
        RETRACTING,
        RESETTING
    }

    State state;
    PDController pidController;
    MotionProfile profile;
    PivotMotor pivotMotor;
    double retractTime, resetTime;
    double manualPower;
    BoxtubeIKPose pose;
    BoxtubeSpline spline;

    ExtensionMotors extension; // reference to extension motor for feedforward

    public Pivot() {
        pivotMotor = new PivotMotor();
        extension = null;

        profile = new MotionProfile(0,0,MAX_VELO, MAX_ACCEL);
        pidController = new PDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        state = State.IDLE;
        manualPower = 0;
    }

    @Override
    public void init() {
        pivotMotor.init();

        state = State.IDLE;
    }

    @Override
    public void read() {
        pivotMotor.read();
    }

    @Override
    public void write() {
        switch (state) {
            case IDLE:
            case MANUAL:
                setRawPower(manualPower);
                manualPower = 0;
                break;
            case IK:
            case PID:
                setPowerFF(pidController.calculate(pivotMotor.getAngle()));
                break;
            case BOXTUBE_SPLINE:
                Vector2d motorState = pivotMotor.getState();
                setPowerFF(pidController.calculate(motorState, limitState(spline.states.pivotState)));
                break;
            case RESETTING:
                setRawPower(0);

                if(Globals.timeSince(resetTime) > 300 && getAngle() < 0.12) {
                    resetEncoder();
                    pidTo(0.0);
                }
                break;
            case RETRACTING:
                double profilePower = pidController.calculate(pivotMotor.getState(), profile);
                setPowerFF(profilePower);

                if(pivotMotor.getAngle() < 0.10 && Math.abs(pivotMotor.getAngleVel()) < 0.1 && Globals.timeSince(retractTime) > 800) {
                    state = State.RESETTING;
                    resetTime = Globals.time();
                }
                break;
        }

        pivotMotor.write();
    }

    public void setIKPose(BoxtubeIKPose pose) {
        state = State.IK;
        pidController.setSetPoint(Range.clip(pose.pivotAngle, MIN_RAD, MAX_RAD));
        this.pose = pose;
    }

    public void followBoxtubeSpline(BoxtubeSpline spline) {
        state = State.BOXTUBE_SPLINE;
        this.spline = spline;
    }

    public void pidTo(double angle) {
        state = State.PID;
        pidController.setSetPoint(Range.clip(angle, MIN_RAD, MAX_RAD));
    }

    public void retract() {
        pidTo(0.0);
        retractTime = Globals.time();
        state = State.RETRACTING;
    }

    public double getAngle() {
        return pivotMotor.getAngle();
    }

    private double getFFNoExtension() {
        return kFF_COS * Math.cos(pivotMotor.getAngle());
    }

    private double getFF(double extensionInches) {
        return Math.cos(pivotMotor.getAngle()) * (kFF_COS + kFF_EXTENSION * extensionInches);
    }

    public void updatePID() {
        pidController.setPID(kP, kI, kD);
    }

    public void setRawPower(double power){
        pivotMotor.setPower(Range.clip(power, MAX_DOWN_POWER, MAX_UP_POWER));
    }

    public void setManualPower(double power) {
        state = State.MANUAL;
        manualPower = power;
    }

    public void setPowerFF(double power) {
        double ff;

        if(extension == null) {
            ff = kFF_COS * Math.cos(pivotMotor.getAngle());
        } else {
            ff = Math.cos(pivotMotor.getAngle()) *
                    (kFF_COS + kFF_EXTENSION * extension.getDistance());
        }
        setRawPower(power + ff);
    }

    public void idle() {
        state = State.IDLE;
        pivotMotor.setPower(0);
    }

    public void useExtension(ExtensionMotors extension) {
        this.extension = extension;
    }

    public PivotMotor getMotor() {
        return pivotMotor;
    }

    public void resetEncoder() {
        pivotMotor.resetEncoder();
        pidController.reset();
    }

    public Vector2d limitState(Vector2d state) {
        double vel = state.getY();

        if(state.getX() < MIN_RAD || state.getX() > MAX_RAD) {
            vel = 0;
        }

        return new Vector2d(Range.clip(state.getX(), MIN_RAD, MAX_RAD), vel);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Pivot State", state);
        pidController.telemetry("Pivot");
        profile.telemetry(telemetry);
        pivotMotor.telemetry();
//        resetLimitSwitch.telemetry();
    }
}
