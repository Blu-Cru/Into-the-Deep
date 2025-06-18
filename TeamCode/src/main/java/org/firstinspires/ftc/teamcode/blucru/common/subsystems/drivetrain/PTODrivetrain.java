package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

public class PTODrivetrain extends Drivetrain implements BluSubsystem, Subsystem {
    public static double
            kP_LEFT = 0.00,
            kI_LEFT = 0.0,
            kD_LEFT = 0.0,
            kP_RIGHT = 0.00,
            kI_RIGHT = 0.0,
            kD_RIGHT = 0.0,

            vMAX = 300.0,
            aMAX = 600.0; // constraints for velocity and acceleration (ticks/s and ticks/s^2)

    boolean isPID;
    BluEncoder leftEncoder, rightEncoder;
    PIDController leftPID, rightPID;
    MotionProfile profile;
    double leftPos, rightPos;

    public PTODrivetrain() {
        super();

        leftEncoder = new BluEncoder("");
        rightEncoder = new BluEncoder("");

        leftPID = new PIDController(kP_LEFT, kI_LEFT, kD_LEFT);
        rightPID = new PIDController(kP_RIGHT, kI_RIGHT, kD_RIGHT);
        profile = new MotionProfile(0, 0, 0, 0);
    }

    @Override
    public void init() {
        super.init();

        isPID = false;

        leftEncoder.reset();
        rightEncoder.reset();
    }

    @Override
    public void read() {
        super.read();

        leftPos = leftEncoder.getCurrentPosition();
        rightPos = rightEncoder.getCurrentPosition();
    }

    @Override
    public void write() {
        if(isPID) {

        } else {
            super.write();
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("PTO Drivetrain Is PID", isPID);
        telemetry.addData("leftPID setpoint", leftPID.getSetPoint());
        telemetry.addData("rightPID setpoint", rightPID.getSetPoint());
        telemetry.addData("Left Hang Encoder", leftPos);
        telemetry.addData("Right Hang Encoder", rightPos);
        super.telemetry(telemetry);
    }
}
