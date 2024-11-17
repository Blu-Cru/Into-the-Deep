package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DrivePID;

public class NewDrivetrain extends DriveBase implements Subsystem {
    enum State {
        IDLE,
        PID
    }

    public boolean fieldCentric = true;
    double drivePower = 0.5;
    State state;
    DrivePID pid;

    public NewDrivetrain() {
        super();
        state = State.IDLE;
        pid = new DrivePID();
    }

    @Override
    public void write() {
        switch (state) {
            case PID:
                driveFieldCentric(DriveKinematics.clip(pid.calculate(getPoseEstimate()), drivePower));
                break;
            case IDLE:
                break;
        }

        super.write();
    }

    public void pidTo(Pose2d targetPose) {
        state = State.PID;
        pid.setTargetPose(targetPose);
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        pid.setTargetHeading(targetHeading);

        if(fieldCentric) {
            driveFieldCentric(new Vector2d(x, y).times(drivePower), pid.getRotate(getHeading()));
        } else {
            drive(new Pose2d(x, y, pid.getRotate(getHeading())));
        }
    }

    public void updatePID() {
        pid.updatePID();
    }

    public void idle() {
        state = State.IDLE;
    }

    public void setDrivePower(double power) {
        drivePower = Range.clip(power, 0.1, 1);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Drivetrain State", state);
        super.telemetry(telemetry);
    }
}
