package org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

@Config
public class Claw extends BluServo implements BluSubsystem, Subsystem {
    public static double
            LOOSE_POS = 0.84, FIRM_POS = 0.89, OPEN_POS = 0.6;

    public Claw() {
        super("claw");
    }

    public void init() {
        super.init();
        grab();
    }

    public void grab() {
        setPosition(FIRM_POS);
    }

    public void release() {
        setPosition(OPEN_POS);
    }

    public void looseGrab() {
        setPosition(LOOSE_POS);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
