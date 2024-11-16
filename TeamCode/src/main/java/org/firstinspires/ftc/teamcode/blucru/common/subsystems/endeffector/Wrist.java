package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

@Config
public class Wrist extends BluServo implements BluSubsystem, Subsystem {
    public static double HORIZONTAL_POS = 0.595;
    public Wrist() {
        super("wrist");
    }

    @Override
    public void init() {
        super.init();
        palmDown();
    }

    // these methods are named based on position of human wrist
    // hold hand in C shape, thumb is bottom, 4 fingers are top where the spinning wheel is

    public void palmDown() {
        setPosition(HORIZONTAL_POS - 0.28);
    }

    public void horizontal() {
        setPosition(HORIZONTAL_POS);
    }

    public void palmUp() {
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
