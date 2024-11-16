package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

@Config
public class Pivot implements BluSubsystem, Subsystem {
    public static double
            kP = 4.5, kI = 0.0, kD = 0.3, tolerance = 0.0,
            kFF_COS = 0.11, kFF_EXTENSION = 0.011,
            MIN_RAD = 0.0, MAX_RAD = 1.9,
            MAX_UP_POWER = 1.0, MAX_DOWN_POWER = -0.75,
            MAX_VELO = 1.0, MAX_ACCEL = 0.5;

    enum State {
        IDLE,
        PID,
        RETRACTING,
        RESETTING
    }

    State state;
    PDController pidController;
    MotionProfile profile;
    PivotMotor pivotMotor;
    LimitSwitch resetLimitSwitch;
    double retractTime, resetTime;

    ExtensionMotor extension; // reference to extension motor for feedforward

    public Pivot() {
        pivotMotor = new PivotMotor();
        extension = null;

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
                break;
            case RETRACTING:
                if(pivotMotor.getAngle() < 0.10 && Math.abs(pivotMotor.getAngleVel()) < 0.1 && Globals.timeSince(retractTime) > 800) {
                    state = State.RESETTING;
                    resetTime = Globals.time();
                }
                break;
            case RESETTING:
                if(Globals.timeSince(resetTime) > 300 && getAngle() < 0.12) {
                    pivotMotor.setCurrentPosition(0);
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
                setPivotPower(0);
                break;
            case PID:
                double pidPower = pidController.calculate(pivotMotor.getAngle());
                setPowerFF(pidPower);
                break;
            case RETRACTING:
                double profilePower = pidController.calculate(pivotMotor.getState(), profile);
                setPowerFF(profilePower);
                break;
        }

        pivotMotor.write();
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
