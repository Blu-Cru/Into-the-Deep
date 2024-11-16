package org.firstinspires.ftc.teamcode.blucru.common.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluMotor extends DcMotorImpl implements BluHardwareDevice {
    String name;
    double power = 0, lastPower = 0;

    public BluMotor(String name, Direction direction) {
        this(name, direction, ZeroPowerBehavior.FLOAT);
    }

    public BluMotor(String name) {
        this(name, Direction.FORWARD, ZeroPowerBehavior.FLOAT);
    }

    public BluMotor(String name, Direction direction, ZeroPowerBehavior zpb) {
        this(Globals.hwMap.get(DcMotor.class, name), name, direction, zpb);
    }

    // main constructor, made private to hide motor object
    private BluMotor(DcMotor motor, String name, Direction direction, ZeroPowerBehavior zpb) {
        super(motor.getController(), motor.getPortNumber(), direction);
        this.name = name;
        super.setZeroPowerBehavior(zpb);
    }

    @Override
    public void init() {
        super.setPower(0);
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        // only update if power has changed
        if(Math.abs(power - lastPower) > 0.005) {
            lastPower = power;
            super.setPower(Globals.correctPower(power));
        }
    }

    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addData(name + " power", power);
    }
}
