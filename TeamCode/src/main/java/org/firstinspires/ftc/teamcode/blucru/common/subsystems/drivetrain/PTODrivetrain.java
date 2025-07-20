package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class PTODrivetrain extends Drivetrain implements BluSubsystem, Subsystem {
    public static double
            kP = 0.4, kI = 0.0, kD = 0.0,

            // negative inches is pulling the robot up more
            TICKS_PER_INCH = 8192.0 / 3.958,

            vMAX = 8.0,
            aMAX = 20.0; // constraints for velocity and acceleration (ticks/s and ticks/s^2)

    enum State {
        PTO_PID,
        PTO_MOTION_PROFILE,
        IDLE
    }

    BluEncoder leftEncoder, rightEncoder;
    PIDController leftPID, rightPID;
    MotionProfile profile;
    double leftPos, rightPos;
    State state;

    public PTODrivetrain() {
        super();

        leftEncoder = new BluEncoder("extension2");
        rightEncoder = new BluEncoder("br", DcMotorSimple.Direction.REVERSE);

        leftPID = new PIDController(kP, kI, kD);
        rightPID = new PIDController(kP, kI, kD);
        profile = new MotionProfile(0, 0, 0, 0).start();
    }

    @Override
    public void init() {
        super.init();

        state = State.IDLE;

        leftEncoder.reset();
        rightEncoder.reset();
    }

    @Override
    public void read() {
        super.read();
        leftEncoder.read();
        rightEncoder.read();

        leftPos = leftEncoder.getCurrentPosition() / TICKS_PER_INCH;
        rightPos = rightEncoder.getCurrentPosition() / TICKS_PER_INCH;
    }

    @Override
    public void write() {
        double leftPower, rightPower;
        switch (state) {
            case PTO_PID:
                leftPower = leftPID.calculate(leftPos);
                rightPower = rightPID.calculate(rightPos);
                setLeftPower(leftPower);
                setRightPower(rightPower);
                break;
            case PTO_MOTION_PROFILE:
                leftPower = leftPID.calculate(profile.getInstantTargetPosition());
                rightPower = rightPID.calculate(profile.getInstantTargetPosition());
                setLeftPower(leftPower);
                setRightPower(rightPower);
                break;
            case IDLE:
                break;
        }
        super.write();
    }

    public void setMotionProfileInches (double inches) {
        state = State.PTO_MOTION_PROFILE;
        profile = new MotionProfile(inches, (leftPos + rightPos) / 2.0, vMAX, aMAX).start();
    }

    public void setLeftPower(double power) {
        bl.setPower(-power);
        fl.setPower(-power);
    }

    public void setRightPower(double power) {
        br.setPower(-power);
        fr.setPower(-power);
    }

    public void stopPID() {
        state = State.IDLE;
        drive(new Pose2d(0,0,0));
    }

    public void updatePID() {
        super.updatePID();
        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
    }

    public void pidTo(double pos) {
        leftPID.setSetPoint(pos);
        rightPID.setSetPoint(pos);
        state = State.PTO_PID;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry(telemetry);
        telemetry.addData("PTO Drivetrain state", state);
        telemetry.addData("leftPID setpoint", leftPID.getSetPoint());
        telemetry.addData("rightPID setpoint", rightPID.getSetPoint());
        telemetry.addData("Left Hang Encoder", leftPos);
        telemetry.addData("Right Hang Encoder", rightPos);
    }
}