package org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.SmoothServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

public class UpDownWrist extends SmoothServo implements BluSubsystem, Subsystem {
    // positive is more down
    public static double PARALLEL_POS = 0.48;
    final double
            TICKS_PER_RAD = 0.275 / (Math.PI / 2),
            MAX_ANGLE = 1.5, MIN_ANGLE = -2.7;

    public UpDownWrist() {
        super("updownwrist", false, 6.0, 15.0);
    }

    @Override
    public void init() {
        super.init();
        retract();
    }

    @Override
    public void read() {
        super.read();
    }

    @Override
    public void write() {
        super.write();
    }

    public void setAngle(double rad) {
        rad = Range.clip(rad, MIN_ANGLE, MAX_ANGLE);
        setPosition(PARALLEL_POS - toTicks(rad));
    }

    public void retract() {
        setAngle(-0.7);
    }

    public double toTicks(double rad) {
        return rad * TICKS_PER_RAD;
    }

    public double toRad(double ticks) {
        return ticks / TICKS_PER_RAD;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
