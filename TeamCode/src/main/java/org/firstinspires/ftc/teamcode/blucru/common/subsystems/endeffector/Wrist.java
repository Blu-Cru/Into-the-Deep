package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

@Config
public class Wrist extends BluServo implements BluSubsystem, Subsystem {
    public static double HORIZONTAL_POS = 0.595,

            TICKS_PER_RAD = 0.28/(Math.PI/2);
    public Wrist() {
        super("wrist");
    }

    @Override
    public void init() {
        super.init();
        front();
    }

    public void front() {
        setPosition(HORIZONTAL_POS - 0.28);
    }

    public void horizontal() {
        setPosition(HORIZONTAL_POS);
    }

    public void back() {
        setPosition(HORIZONTAL_POS + 0.28);
    }

    public void opposite() {
        setPosition(HORIZONTAL_POS - 0.56);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
