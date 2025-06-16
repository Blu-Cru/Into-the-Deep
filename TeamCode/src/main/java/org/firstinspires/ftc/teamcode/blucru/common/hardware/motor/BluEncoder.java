package org.firstinspires.ftc.teamcode.blucru.common.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluEncoder extends DcMotorImplEx implements BluHardwareDevice {
    String name;
    double encoderTicks = 0, vel = 0;
    double offsetTicks = 0;
    double lastVelTime, lastPos;

    public BluEncoder(String name, Direction direction) {
        this(name, direction, ZeroPowerBehavior.FLOAT);
    }

    public BluEncoder(String name) {
        this(name, Direction.FORWARD, ZeroPowerBehavior.FLOAT);
    }

    public BluEncoder(String name, Direction direction, ZeroPowerBehavior zeroPowerBehavior) {
        this(Globals.hwMap.get(DcMotor.class, name), name, direction, zeroPowerBehavior);
    }

    // main constructor, made private to hide motor object
    private BluEncoder(DcMotor motor, String name, Direction direction, ZeroPowerBehavior zeroPowerBehavior) {
        super(motor.getController(), motor.getPortNumber(), direction);
        this.name = name;
        super.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void init() {
        resetEncoder();
        super.setPower(0);
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

    }

    public void resetEncoder() {
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(RunMode.RUN_WITHOUT_ENCODER);
        offsetTicks = 0;
    }

    public void setCurrentPosition(double pos) {
        offsetTicks = pos - encoderTicks;
    }

    public int getCurrentPosition() {
        return (int) (encoderTicks + offsetTicks);
    }

    @Override
    public double getVelocity() {
        return vel;
    }

    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addData(name + " pos: ", encoderTicks);
        tele.addData(name + " vel: ", vel);
    }
}
