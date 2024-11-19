package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DrivePID;

public class Drivetrain extends DriveBase implements Subsystem {
    public static double PATH_HEADING_TOLERANCE = 0.18;

    enum State {
        IDLE,
        PID
    }

    public boolean fieldCentric = true;
    public double drivePower = 0.5;
    State state;
    DrivePID pid;

    boolean lastTurning, lastTranslating;

    public Drivetrain() {
        super();
        state = State.IDLE;
        pid = new DrivePID();

        lastTurning = false;
        lastTranslating = false;
    }

    @Override
    public void init() {
        super.init();

        pid.setTargetPose(pose);
    }

    @Override
    public void read() {
        super.read();
    }

    @Override
    public void write() {
        switch (state) {
            case PID:
                driveFieldCentric(DriveKinematics.clip(pid.calculate(pose), drivePower));
                break;
            case IDLE:
                break;
        }

        super.write();
    }

    // call this every loop
    public void teleOpDrive(Gamepad g1) {
        state = State.IDLE;

        double vert = -g1.left_stick_y;
        double horiz = g1.left_stick_x;
        double rotate = -g1.right_stick_x;

        if(g1.a) {
            driveToHeading(horiz, vert, Math.PI/4);
            return;
        }

        boolean translating = Math.abs(vert) > 0.05 || Math.abs(horiz) > 0.05;
        boolean turning = Math.abs(rotate) > 0.05;

        if(turning) {
            // drive normally
            driveFieldCentric(new Vector2d(horiz, vert), rotate);
        } else if(lastTurning) {
            // if just turning, turn to new heading
            pid.headingController.reset();
            driveToHeading(horiz, vert, DriveKinematics.getHeadingDecel(heading, headingVel));
        } else if(translating && !lastTranslating) {
            // if just started translating, drive to current heading
            driveToHeading(horiz, vert, heading);
        } else if(!translating) {
            pid.setTargetHeading(heading);
            // if not translating, drive 0,0,0
            drive(new Pose2d(0,0,0));
        } else {
            // if translating and not turning, drive to target heading
            driveToHeading(horiz, vert);
        }

        lastTurning = turning;
        lastTranslating = translating;
    }

    public void pidTo(Pose2d targetPose) {
        state = State.PID;
        pid.setTargetPose(targetPose);
    }

    public void driveToHeading(double x, double y) {
        if(fieldCentric) {
            driveFieldCentric(new Vector2d(x, y).times(drivePower), pid.getRotate(heading));
        } else {
            drive(new Vector2d(x, y).times(drivePower), pid.getRotate(heading));
        }
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        pid.setTargetHeading(targetHeading);

        driveToHeading(x, y);
    }

    public void updatePID() {
        pid.updatePID();
    }

    public void idle() {
        state = State.IDLE;
        drive(new Pose2d(0,0,0));
    }

    public boolean inRange(double translationTolerance) {
        return pid.inRange(pose, translationTolerance, PATH_HEADING_TOLERANCE);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Drivetrain State", state);
        super.telemetry(telemetry);
    }
}
