package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

@Config
public class Pivot implements Subsystem {
    public static double
            kP = 4.5, kI = 0.0, kD = 0.3, tolerance = 0.0,
            kFF_COS = 0.13, kFF_EXTENSION = 0.011,
            MIN_RAD = 0.0, MAX_RAD = 1.9,
            MAX_UP_POWER = 1.0, MAX_DOWN_POWER = -0.75,
            MAX_VELO = 1.0, MAX_ACCEL = 0.5;

    enum State {
        IDLE,
        PID,
//        MOTION_PROFILE,
        RETRACTING,
        RESETTING
    }

    State state;
    PDController pidController;
    MotionProfile profile;
    PivotMotor pivotMotor;
    LimitSwitch resetLimitSwitch;
    ElapsedTime resetTimer;

    ExtensionMotor extension; // reference to extension motor for feedforward

    public Pivot() {
        pivotMotor = new PivotMotor();
        extension = null;

        resetTimer = new ElapsedTime();
        profile = new MotionProfile(0,0,MAX_VELO, MAX_ACCEL);
        pidController = new PDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        state = State.IDLE;
    }

    @Override
    public void init() {
        pivotMotor.init();

        state = State.IDLE;
    }

    @Override
    public void read() {
        pivotMotor.read();

        switch(state) {
            case IDLE:
            case PID:
//            case MOTION_PROFILE:
                break;
            case RETRACTING:
                if(profile.done() && Math.abs(pivotMotor.getAngle()) < 0.1 && Math.abs(pivotMotor.getAngleVel()) < 0.3) {
                    state = State.RESETTING;
                    resetTimer.reset();
                }
                break;
            case RESETTING:
                if(resetTimer.seconds() > 0.3 && getAngle() < 0.12) {
                    pivotMotor.resetEncoder();
                    pidTo(0.0);
                }
                break;
        }
    }

    @Override
    public void write() {
        switch (state) {
            case IDLE:
                break;
            case RESETTING:
                setPivotPower(-0.05);
                break;
            case PID:
                double pidPower = pidController.calculate(pivotMotor.getAngle());
                setPowerFF(pidPower);
                break;
//            case MOTION_PROFILE:
            case RETRACTING:
                double profilePower = pidController.calculate(pivotMotor.getState(), profile);
                setPowerFF(profilePower);
                break;
        }

        pivotMotor.write();
    }

//    public void setMotionProfileTargetAngle(double targetAngle) {
//        targetAngle = Range.clip(targetAngle, MIN_RAD, MAX_RAD);
//        state = State.MOTION_PROFILE;
//        profile = new MotionProfile(targetAngle, pivotMotor.getAngle(), MAX_VELO, MAX_ACCEL).start();
//    }

    public void pidTo(double angle) {
        state = State.PID;
        pidController.setSetPoint(Range.clip(angle, MIN_RAD, MAX_RAD));
    }

    public void retract() {
        state = State.RETRACTING;
        pidTo(0.05);
//        setMotionProfileTargetAngle(0.08);
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

    public void setTargetAngle(double angleRad) {
        pidController.setSetPoint(Range.clip(angleRad, MIN_RAD, MAX_RAD));
    }

    public void updatePID() {
        pidController.setPID(kP, kI, kD);
    }

    private void setPivotPower(double power){
        pivotMotor.setPower(Range.clip(power, MAX_DOWN_POWER, MAX_UP_POWER));
    }

    public void setPowerFF(double power) {
        double ff;

        if(extension == null) {
            ff = getFFNoExtension();
        } else {
            ff = getFF(extension.getDistance());
        }
        setPivotPower(power + ff);
    }

    public void idle() {
        state = State.IDLE;
        pivotMotor.setPower(0);
    }

    public void useExtension(ExtensionMotor extension) {
        this.extension = extension;
    }

    public PivotMotor getMotor() {
        return pivotMotor;
    }

    public void resetEncoder() {
        pivotMotor.resetEncoder();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Pivot State", state);
        profile.telemetry(telemetry);
        pivotMotor.telemetry();
//        resetLimitSwitch.telemetry();
    }
}
