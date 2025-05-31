package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

@Config
public class Claw extends BluServo implements BluSubsystem, Subsystem {
    public static double
            LOOSE_POS = 0.44, FIRM_POS = LOOSE_POS - 0.06, OPEN_POS = LOOSE_POS + 0.21;

    public Claw() {
        super("clamp");
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
