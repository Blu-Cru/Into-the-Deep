package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DrivePID;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.TurnToBlockKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class Drivetrain extends DriveBase implements Subsystem {
    public static double PATH_HEADING_TOLERANCE = 0.18;

    enum State {
        IDLE,
        PID,
        PID_Y_HEADING
    }

    public boolean fieldCentric = true;
    double drivePower = 0.5;
    State state;
    DrivePID pid;
    TurnToBlockKinematics blockKinematics;

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

//        setPoseEstimate(Globals.startPose);

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
                driveFieldCentric(DriveKinematics.clip(pid.calculate(this), drivePower));
                break;
            case IDLE:
            case PID_Y_HEADING:
                break;
        }

        super.write();
    }

    // call this every loop
    public void teleOpDrive(Gamepad g1) {
        state = State.IDLE;

        double vert = -g1.left_stick_y;
        double horiz = g1.left_stick_x;
        double rotate = -g1.right_stick_x * 0.8;

        boolean translating = Math.abs(vert) > 0.05 || Math.abs(horiz) > 0.05;
        boolean turning = Math.abs(rotate) > 0.05;

        if(turning) {
            // drive normally
            driveFieldCentric(new Vector2d(horiz, vert).times(drivePower), rotate * drivePower, Globals.alliance);
        } else if(lastTurning) {
            // if just turning, turn to new heading
            pid.headingController.reset();
            driveToHeading(horiz, vert, DriveKinematics.getHeadingDecel(heading, headingVel), Globals.alliance);
        } else if(translating && !lastTranslating) {
            // if just started translating, drive to current heading
            driveToHeading(horiz, vert, heading, Globals.alliance);
        } else if(!translating) {
            pid.headingController.reset();
            pid.setTargetHeading(heading);
            // if not translating, drive 0,0,0
            drive(new Pose2d(0,0,0));
        } else {
            // if translating and not turning, drive to target heading
            driveToHeading(horiz, vert, Globals.alliance);
        }

        lastTurning = turning;
        lastTranslating = translating;
    }

    public void driveToYHeading(double xInput, double ySetPoint, double headingSetPoint) {
        pid.setTargetPose(new Pose2d(xInput, ySetPoint, headingSetPoint));
    }

    public void pidTo(Pose2d targetPose) {
        state = State.PID;
        pid.setTargetPose(targetPose);
    }

    public void teleDriveTurnToPos(double x, double y, Vector2d pos, boolean useVel) {
        state = State.IDLE;
        blockKinematics = new TurnToBlockKinematics(pos);

        double rotate;
        if(useVel) {
            Vector2d pv = new Vector2d(heading, headingVel);
            Vector2d dtHeadingState = blockKinematics.getHeadingStateTowardsPoint(pose, vel);
            Vector2d sp = new Vector2d(dtHeadingState.getX(), - dtHeadingState.getY());

            rotate = pid.getRotate(pv, sp);
        } else {
            pid.setTargetHeading(blockKinematics.getHeadingTowardsPoint(pose));
            rotate = pid.getRotate(headingState);
        }

        driveFieldCentric(new Vector2d(x, y).times(drivePower), rotate);
    }

    public void driveToHeading(double x, double y) {
        if(fieldCentric) {
            driveFieldCentric(new Vector2d(x, y).times(drivePower), pid.getRotate(this));
        } else {
            drive(new Vector2d(x, y).times(drivePower), pid.getRotate(this));
        }
    }

    public void driveToHeading(double x, double y, Alliance alliance) {
        if(fieldCentric) {
            driveFieldCentric(new Vector2d(x, y).times(drivePower), pid.getRotate(this), alliance);
        } else {
            drive(new Vector2d(x, y).times(drivePower), pid.getRotate(this));
        }
    }

    // TODO: implement this
    public void driveTurnToBlock(Vector2d blockPos) {
        blockKinematics = new TurnToBlockKinematics(blockPos);
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        pid.setTargetHeading(targetHeading);

        driveToHeading(x, y);
    }

    public void driveToHeading(double x, double y, double targetHeading, Alliance alliance) {
        pid.setTargetHeading(targetHeading);

        driveToHeading(x, y, alliance);
    }

    public void pidYHeadingMapped(double x, double y, double heading) {
        Pose2d mapped = Globals.mapPose(0, y, Math.toDegrees(heading));
        y = mapped.getY();
        heading = mapped.getHeading();

        fieldCentric = true;
        pid.yController.setSetPoint(y);
        double yOutput = pid.yController.calculate(yState);

        driveToHeading(x * Globals.reflect, yOutput, heading);
    }

    public void updatePID() {
        pid.updatePID();
    }

    public void idle() {
        state = State.IDLE;
        drive(new Pose2d(0,0,0));
    }

    public Pose2d getStopPose() {
        return DriveKinematics.getStopPose(pose, vel);
    }

    public boolean inRange(double translationTolerance, double headingTolerance) {
        return pid.inRange(pose, translationTolerance, headingTolerance);
    }

    public void setDrivePower(double power) {
        drivePower = Globals.correctPower(power);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Drivetrain State", state);
        telemetry.addData("Drive Power", drivePower);
        telemetry.addData("heading", heading);
        super.telemetry(telemetry);
    }
}
