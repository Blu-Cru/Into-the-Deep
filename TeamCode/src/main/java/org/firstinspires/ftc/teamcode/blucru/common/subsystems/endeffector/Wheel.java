package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;

public class Wheel extends BluCRServo implements BluSubsystem, Subsystem {
    public Wheel() {
        super("wheel");
    }

    public void intake() {
        super.setPower(-1);
    }

    public void reverse() {
        super.setPower(1);
    }

    public void stop() {
        super.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry();
    }
}
