package org.firstinspires.ftc.teamcode.blucru.common.hardware.servo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;


// all the hardware wrapper classes need testing
public class BluCRServo extends CRServoImpl implements BluHardwareDevice {
    String name;
    double power, lastPower;

    public BluCRServo(String name, Direction direction) {
        this(Globals.hwMap.get(CRServo.class, name), name, direction);
        power = 0;
        lastPower = 0;
    }

    public BluCRServo(String name) {
        this(name, Direction.FORWARD);
    }

    private BluCRServo(CRServo servo, String name, Direction direction) {
        super(servo.getController(), servo.getPortNumber(), direction);
        this.name = name;
        power = 0;
        lastPower = 0;
    }

    @Override
    public void init() {
        super.setPower(0);
        setPower(0);
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        if(Math.abs(power - lastPower) > 0.003) {
            super.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public void setPower(double power) {
        this.power = Range.clip(power, -1, 1);
    }

    @Override
    public void telemetry() {
        Globals.tele.addLine(name + " power: " + power);
    }
}
