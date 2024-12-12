package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

@Config
public class Wrist extends BluServo implements BluSubsystem, Subsystem {
    public static double HORIZONTAL_POS = 0.595,
            MIN_ANGLE = -Math.PI, MAX_ANGLE = Math.PI/2,

            TICKS_PER_RAD = 0.28/(Math.PI/2);
    public Wrist() {
        super("wrist");
    }

    @Override
    public void init() {
        super.init();
        front();
    }

    @Override
    public void write() {
        super.write();
    }

    public void setAngle(double angle) {
        angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        setPosition(HORIZONTAL_POS + angle * TICKS_PER_RAD);
    }

    public double getAngle() {
        return (getPosition() - HORIZONTAL_POS) / TICKS_PER_RAD;
    }

    public void front() {
        setAngle(-Math.PI/2);
    }

    public void horizontal() {
        setAngle(0);
    }

    public void back() {
        setAngle(Math.PI/2);
    }

    public void opposite() {
        setAngle(-Math.PI);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Wrist Angle", getAngle());
    }
}
