package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

@Config
public class HangMotor implements BluSubsystem, Subsystem {
    public static double kP = 0.004, kI = 0.0, kD = 0.0;
    enum State {
        IDLE,
        PID,
        MANUAL
    }
    State state;

    // low hang: 11253
    // high hooks on bar: -2200

    BluMotorWithEncoder motor;
    PDController pid;
    double manualPower;

    public HangMotor() {
        motor = new BluMotorWithEncoder("hang", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        state = State.IDLE;
        pid = new PDController(kP, kI, kD);
    }

    @Override
    public void init() {
        motor.init();

        state = State.IDLE;
    }

    @Override
    public void read() {
        motor.read();
    }

    @Override
    public void write() {
        switch (state) {
            case IDLE:
                motor.setPower(0);
                break;
            case PID:
                motor.setPower(pid.calculate(motor.getCurrentPosition()));
                break;
            case MANUAL:
                motor.setPower(manualPower);
                manualPower = 0;
                break;
        }
        motor.write();
    }

    public void pidHighBar(){
        pidTo(-2000);
    }

    public void pidTo(double ticks) {
        state = State.PID;
        pid.setSetPoint(ticks);
    }

    public void holdPosition() {
        pidTo(motor.getCurrentPosition());
    }

    public void idle() {
        state = State.IDLE;
    }

    public void updatePID() {
        pid.setPID(kP, kI, kD);
    }

    public void setManualPower(double power) {
        if(Math.abs(power) > 0.1) {
            state = State.MANUAL;
            manualPower = power;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Hang state", state);
        motor.telemetry();
    }
}
