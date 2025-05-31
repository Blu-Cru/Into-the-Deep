package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.SmoothServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

public class UpDownWrist extends SmoothServo implements BluSubsystem, Subsystem {
    // positive is more down
    public static double PARALLEL_POS = 0.435;
    final double TICKS_PER_RAD = 0.275 / (Math.PI / 2);

    public UpDownWrist() {
        super("updownwrist", false);
    }

    @Override
    public void read() {
        super.read();
    }

    @Override
    public void write() {
        super.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
