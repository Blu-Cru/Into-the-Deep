package org.firstinspires.ftc.teamcode.blucru.common.hardware.servo;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluServo extends ServoImpl implements BluHardwareDevice {
    String name;
    ServoController controller;
    double pos=0, lastPos=0;

    public BluServo(String name, Direction direction) {
        this(Globals.hwMap.get(ServoImpl.class, name), name, direction);
    }

    public BluServo(String name) {
        this(name, Direction.FORWARD);
    }

    public BluServo(String name, boolean reversed) {
        this(name, reversed ? Direction.REVERSE : Direction.FORWARD);
    }

    private BluServo(ServoImpl servo, String name, Direction direction) {
        super(servo.getController(), servo.getPortNumber(), servo.getDirection());
        this.setDirection(direction);
        this.controller = servo.getController();
        this.name = name;
    }

    public void setPosition(double position) {
        pos = Range.clip(position, 0, 1);
    }

    public double getPosition() {
        return pos;
    }

    /*
        * Enable/disable the servo
        *
        * WATCH OOUT!!!!, each servo controller is connected to 2 servo ports,
        * so using enable or disable might affect other servos
     */
    public void enable() {
        controller.pwmEnable();
    }

    public void disable() {
        controller.pwmDisable();
    }

    public void init() {
    }

    public void read() {

    }

    public void write() {
        if(Math.abs(pos - lastPos) > 0.002) {
            lastPos = pos;
            super.setPosition(pos);
        }
    }

    public String getName() {
        return name;
    }

    public void telemetry() {
        try {
            Globals.tele.addLine(name + " pos: " + pos);
        } catch (Exception ignored) {}
    }

    public void addLine(String str) {
        try {
            Globals.tele.addLine(name + " " + str);
        } catch (Exception ignored) {}
    }
}
