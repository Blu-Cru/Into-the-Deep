package org.firstinspires.ftc.teamcode.blucru.common.hardware.servo;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

public class SmoothServo extends BluServo implements BluHardwareDevice {
    static final double defaultVmax = 3, defaultAmax = 4;

    MotionProfile motionProfile;
    double finalPosition, currentPosition;
    double vMax, aMax;

    public SmoothServo(String name, boolean reversed) {
        this(name, reversed, defaultVmax, defaultAmax);
    }

    public SmoothServo(String name) {
        this(name, false, defaultVmax, defaultAmax);
    }

    public SmoothServo(String name, boolean reversed, double vMax, double aMax) {
        super(name, reversed);
        this.vMax = vMax;
        this.aMax = aMax;
    }

    public void init() {
        super.init();
    }

    public void read() {
        super.read();
    }

    public void write() {
        try {
            currentPosition = motionProfile.getInstantTargetPosition();
            super.setPosition(currentPosition);
        } catch (Exception e) {

        }
        super.write();
    }

    public void setPosition(double position) {
        finalPosition = position;
        if(motionProfile == null) currentPosition = position;
        motionProfile = new MotionProfile(finalPosition, currentPosition, vMax, aMax).start();
    }

    public void setConstraints(double vMax, double aMax) {
        this.vMax = vMax;
        this.aMax = aMax;
    }

    public void telemetry() {
        Globals.tele.addData("Target Pos: ", finalPosition);
        super.telemetry();
    }
}