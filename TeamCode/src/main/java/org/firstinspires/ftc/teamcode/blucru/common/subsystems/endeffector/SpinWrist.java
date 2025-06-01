package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class SpinWrist extends BluServo implements BluSubsystem, Subsystem {
    public static double
            // positive turns clockwise when intaking
            CENTER_POS = 0.38;
    static final double
            // 1 tick = range/360 of a full rotation
            TICKS_PER_RAD = 0.28/(Math.PI/2),
            MAX_ANGLE = Math.PI/2, MIN_ANGLE = -Math.PI;

    enum State {
        SERVO,
        TURRET_IK
    }

    State state;
    double globalAngle;

    public SpinWrist() {
        super("spinwrist");
        state = State.SERVO;
    }

    @Override
    public void init() {
        super.init();
        center();
    }

    @Override
    public void read() {
        super.read();
    }

    @Override
    public void write() {
        switch(state) {
            case TURRET_IK:
                setAngle(Range.clip(globalAngle - Robot.getInstance().turret.getAngle(), MIN_ANGLE, MAX_ANGLE));
                break;
            case SERVO:
            default:
                // Do nothing, just use the set position
                break;
        }
        super.write();
    }

    public void center() {
        state = State.SERVO;
        setPosition(CENTER_POS);
    }

    public void setAngle(double rad) {
        state = State.SERVO;
        double angle = Range.clip(rad, MIN_ANGLE, MAX_ANGLE);
        super.setPosition(toTicks(normalizeAngle(angle)) + CENTER_POS);
    }

    double normalizeAngle(double rad) {
        while (rad < MIN_ANGLE) {
            rad += Math.PI;
        }

        while (rad > MAX_ANGLE) {
            rad -= Math.PI;
        }

        return rad;
    }

    public void setTurretGlobalAngle(double globalAngle) {
        state = State.TURRET_IK;
        this.globalAngle = globalAngle;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("SPIN WRIST State", state);
        telemetry.addData("Spin Wrist globalAngle", globalAngle);
        super.telemetry();
    }

    double toTicks(double rad) {
        return rad * TICKS_PER_RAD; // Convert radians to a value between 0 and 1
    }
}
