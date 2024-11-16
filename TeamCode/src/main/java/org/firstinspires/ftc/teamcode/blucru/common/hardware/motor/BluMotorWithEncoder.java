package org.firstinspires.ftc.teamcode.blucru.common.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluMotorWithEncoder extends DcMotorImplEx implements BluHardwareDevice {
    String name;
    double power = 0, lastPower = 0;
    double encoderTicks = 0, vel = 0;
    double offsetTicks = 0;
    double lastVelTime, lastPos;

    public BluMotorWithEncoder(String name, Direction direction) {
        this(name, direction, ZeroPowerBehavior.FLOAT);
    }

    public BluMotorWithEncoder(String name) {
        this(name, Direction.FORWARD, ZeroPowerBehavior.FLOAT);
    }

    public BluMotorWithEncoder(String name, Direction direction, ZeroPowerBehavior zeroPowerBehavior) {
        this(Globals.hwMap.get(DcMotor.class, name), name, direction, zeroPowerBehavior);
    }

    // main constructor, made private to hide motor object
    private BluMotorWithEncoder(DcMotor motor, String name, Direction direction, ZeroPowerBehavior zeroPowerBehavior) {
        super(motor.getController(), motor.getPortNumber(), direction);
        this.name = name;
        super.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPower(double power) {
        // clip power to -1 to 1
        this.power = Range.clip(power, -1, 1);
    }

    public void init() {
        super.setPower(0);
        resetEncoder();
        offsetTicks = 0;
        lastVelTime = System.currentTimeMillis();
        lastPos = encoderTicks;
    }

    public void read() {
        // only update if encoder is being used
        encoderTicks = super.getCurrentPosition();
        vel = super.getVelocity();
    }

    public void write() {
        // only update if power has changed
        if(Math.abs(power - lastPower) > 0.005) {
            lastPower = power;
            super.setPower(Globals.correctPower(power));
        }
    }

    public void resetEncoder() {
        setCurrentPosition(0);
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setCurrentPosition(double pos) {
        offsetTicks = pos - encoderTicks;
    }

    public int getCurrentPosition() {
        return (int) (encoderTicks + offsetTicks);
    }

    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addData(name + " power", power);
        tele.addLine(name + " pos: " + encoderTicks);
        tele.addLine(name + " vel: " + vel);
    }
}
