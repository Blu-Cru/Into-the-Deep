package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;


import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DrivePID;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.FusedLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.util.*;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class Drivetrain extends SampleMecanumDrive implements Subsystem {
    public static double
            MAX_ACCEL_DRIVE_DELTA = 6.5, MAX_DECEL_DRIVE_DELTA = 20.0, // magnitude per second at power 1 for slew rate limiter
            MAX_ACCEL_PID_DELTA = 8, // magnitude per second at power 1 for PID

            HEADING_DECELERATION = 10, // radians per second squared, for calculating new target heading after turning
            HEADING_P = 1.9, HEADING_I = 0, HEADING_D = 0.06, HEADING_PID_TOLERANCE = 0.0, // radians
            HEADING_AT_POSE_TOLERANCE = 0.15,

            TRANSLATION_P = 0.18, TRANSLATION_I = 0, TRANSLATION_D = 0.029, TRANSLATION_PID_TOLERANCE = 0, // PID constants for translation
            TRANSLATION_AT_POSE_TOLERANCE = 0.55,

            STRAFE_kStatic = 0.05, FORWARD_kStatic = 0.02; // feedforward constants for static friction

    enum State {
        TELEOP,
        DRIVE_TO_POSITION,
        FOLLOWING_TRAJECTORY
    }

    State drivetrainState;
    public double drivePower = 0.5;
    double dt, lastTime;

    DrivePID translationPID;
    public Pose2d targetPose;
    public FusedLocalizer fusedLocalizer;

    PIDController headingPID;
    double targetHeading = 0;
    public boolean fieldCentric; // whether the robot is field centric or robot centric

    Vector2d lastDriveVector; // drive vector in previous loop
    double lastRotateInput; // rotate input in previous loop

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.drivetrainState = State.TELEOP;
        headingPID = new PIDController(HEADING_P, HEADING_I, HEADING_D);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);

        translationPID = new DrivePID(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D, TRANSLATION_PID_TOLERANCE);
        fusedLocalizer = new FusedLocalizer(getLocalizer(), hardwareMap);
        lastDriveVector = new Vector2d(0,0);

        fieldCentric = true;
        targetPose = getPoseEstimate();
        lastRotateInput = 0;
        drivetrainState = State.TELEOP;
    }

    public void init() {
        initializePose();

        this.drivetrainState = State.TELEOP;

        fusedLocalizer.init();
    }

    public void read() {
        dt = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();

        updatePoseEstimate();
    }

    public void write() {
        switch(drivetrainState) {
            case TELEOP:
                break;
            case DRIVE_TO_POSITION:
                driveToPosition(targetPose);
                break;
            case FOLLOWING_TRAJECTORY:
                updateTrajectory();
                break;
        }
    }

    public void teleOpDrive(double x, double y, double rotate) {
        drivetrainState = State.TELEOP;

        boolean driving = Math.abs(x) > 0.02 || Math.abs(y) > 0.02 || Math.abs(rotate) > 0.02;
        boolean turning = Math.abs(rotate) > 0.02;
        boolean wasJustTurning = Math.abs(lastRotateInput) > 0.02;

        if(!driving)
            setWeightedDrivePower(new Pose2d(0, 0, 0));
        else if(turning) // if driver is turning, drive with turning normally
            driveScaled(x, y, rotate);
        else if(wasJustTurning) // if driver just stopped turning, drive to new target heading
            driveToHeadingScaled(x, y, calculateHeadingDecel());
        else // drive, turning to target heading
            driveToHeadingScaled(x, y, targetHeading);

        // recording last turn input
        lastRotateInput = rotate;
    }

    public void teleOpDrive(Gamepad g1) {
        double x = g1.left_stick_x;
        double y = -g1.left_stick_y;
        double rotate = -g1.right_stick_x;

        teleOpDrive(x, y, rotate);
    }

    public void driveScaled(double x, double y, double rotate) {
        Vector2d driveVector = rotateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = scaleByDrivePower(new Pose2d(driveVector, rotate));
        Pose2d staticDrivePose = processStaticFriction(drivePose);

        setWeightedDrivePower(staticDrivePose);
    }

    public void driveClip(double x, double y, double rotate) {
        Vector2d driveVector = rotateDriveVector(new Vector2d(x, y));

//        driveVector = limitPIDDriveVector(driveVector);

        Pose2d drivePose = clipByDrivePower(new Pose2d(driveVector, rotate));
        Pose2d staticDrivePose = processStaticFriction(drivePose);

        setWeightedDrivePower(staticDrivePose);
    }

    public void driveToHeadingScaled(double x, double y, double targetHeading) {
        this.targetHeading = targetHeading;
        double rotate = getHeadingPID(targetHeading);

        Vector2d driveVector = rotateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = new Pose2d(driveVector.times(drivePower), Range.clip(rotate, -drivePower, drivePower));
        Pose2d staticDrivePose = processStaticFriction(drivePose);

        setWeightedDrivePower(staticDrivePose);
    }

    public void driveToHeadingClip(double x, double y, double targetHeading) {
        this.targetHeading = targetHeading;
        double rotate = getHeadingPID(targetHeading);
        
        driveClip(x, y, rotate);
    }

    private Vector2d rotateDriveVector(Vector2d input) {
        if (fieldCentric) return input.rotated(-getExternalHeading()); // rotate input vector to match robot heading if field centric
        else return input.rotated(Math.toRadians(-90)); // rotate to match robot coordinates (x forward, y left)
    }

    private Vector2d limitPIDDriveVector(Vector2d input) {
        if(input.norm() <= lastDriveVector.norm() || input.norm() == 0) return input;
        else {
            // limit to the maximum change in the drive vector per second at the drive power
            double newMagnitude = Math.min(Math.min(input.norm(), lastDriveVector.norm() + MAX_ACCEL_PID_DELTA * dt / 1000.0), 1);

            Vector2d driveVector = input.div(input.norm()).times(newMagnitude);
            lastDriveVector = driveVector;
            return driveVector;
        }
    }

    private Pose2d processStaticFriction(Pose2d drivePose) {
        Vector2d driveVector = drivePose.vec();
//        boolean robotStopped = velocity.vec().norm() < STATIC_TRANSLATION_VELOCITY_TOLERANCE && Math.abs(velocity.getHeading()) < STATIC_HEADING_VELOCITY_TOLERANCE;

//        if(robotStopped && driveVector.norm() != 0) {
            double angle = driveVector.angle();
            double staticMinMagnitude =
                    STRAFE_kStatic * FORWARD_kStatic
                            /
                    Math.hypot(STRAFE_kStatic * Math.cos(angle), FORWARD_kStatic * Math.sin(angle));
            double newDriveMagnitude = staticMinMagnitude + (1-staticMinMagnitude) * driveVector.norm();
            return new Pose2d(driveVector.div(driveVector.norm()).times(newDriveMagnitude), drivePose.getHeading());
//        } else return drivePose;
    }

    private Pose2d scaleByDrivePower(Pose2d drivePose) {
        return drivePose.times(drivePower);
    }

    private Pose2d clipByDrivePower(Pose2d drivePose) {
        double newX = Range.clip(drivePose.getX(), -drivePower, drivePower);
        double newY = Range.clip(drivePose.getY(), -drivePower, drivePower);
        double newHeading = Range.clip(drivePose.getHeading(), -drivePower, drivePower);
        return new Pose2d(newX, newY, newHeading);
    }

    public void driveToPosition(Pose2d targetPosition) {
        translationPID.setTargetPosition(targetPosition.vec());
        Vector2d rawDriveVector = translationPID.calculate(getPoseEstimate().vec());

        driveToHeadingClip(rawDriveVector.getX(), rawDriveVector.getY(), targetPosition.getHeading());
    }

    // set the component of a vector in a direction
    private Vector2d setComponent(Vector2d vector, double component, double angle) {
        vector = vector.rotated(-angle); // rotate so the component is in the x direction
        vector = new Vector2d(component, vector.getY()); // set the x component
        return vector.rotated(angle); // rotate back
    }

    public double calculateHeadingDecel() {
        // vf^2 = vi^2 + 2a(xf - xi)
        // 0 = velocity * velocity + 2 * -HEADING_DECELERATION * (targetHeading - heading)
        // velocity * velocity = 2 * HEADING_DECELERATION * (targetHeading - heading)
        // target heading = heading + 0.5 * velocity * velocity / HEADING_DECELERATION
        // use sign of velocity to determine to add or subtract

        return Angle.norm(getExternalHeading() + Math.signum(getExternalHeadingVelocity()) * 0.5 * getExternalHeadingVelocity() * getExternalHeadingVelocity() / HEADING_DECELERATION);
    }

    public void idle() {
        drivetrainState = State.TELEOP;
        lastDriveVector = new Vector2d(0,0);
    }

    public void pidTo(Pose2d pose) {
        fieldCentric = true;
        drivetrainState = State.DRIVE_TO_POSITION;
        pose = new Pose2d(pose.getX(), pose.getY(), Angle.norm(pose.getHeading()));
        setTargetPose(pose);
    }

    public void setDrivePower(double power) {
        drivePower = Range.clip(power, 0.15, 1.0);
    }

    public void updateTurnPID() {
        headingPID.setPID(HEADING_P, HEADING_I, HEADING_D);
    }

    public void updateTranslationPID() {
        translationPID.setPID(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
    }

    private double getHeadingPID(double target) {
        double heading = getExternalHeading();

        if(heading - target < -Math.PI) heading += 2*Math.PI;
        else if(heading - target > Math.PI) heading -= 2 * Math.PI;

        if(Math.abs(heading - target) < HEADING_PID_TOLERANCE) return 0;
        else return Range.clip(headingPID.calculate(heading, target), -1, 1);
    }

    public double getAbsHeadingError(double target) {
        double error = getExternalHeading() - target;
        if(error < -Math.PI) error += 2*Math.PI;
        else if(error > Math.PI) error -= 2*Math.PI;
        return Math.abs(error);
    }

    public double getAbsHeadingError() {
        return getAbsHeadingError(targetHeading);
    }

    public double getOdoHeading() {
        double heading = getPoseEstimate().getHeading();
        if(heading > Math.PI) {
            heading -= 2*Math.PI;
        } else if(heading < -Math.PI) {
            heading += 2*Math.PI;
        }
        return heading;
    }

    public void setTeleDrivePower(Gamepad gamepad) {
        boolean slow = gamepad.left_trigger > 0.1,
                fast = gamepad.right_trigger > 0.1;

        double slowPower, fastPower, normalPower;
//        switch (robotState) {
//            case RETRACT: slowPower = 0.4; fastPower = 1.0; normalPower = 0.9; break;
//            case INTAKING: slowPower = 0.3; fastPower = 1.0; normalPower = 0.85; break;
//            case LIFTING: slowPower = 0.3; fastPower = 0.7; normalPower = 0.6; break;
//            case OUTTAKING: slowPower = 0.25; fastPower = 0.8; normalPower = 0.5; break;
//            case OUTTAKE_WRIST_RETRACTED: slowPower = 0.3; fastPower = 0.7; normalPower = 0.7; break;
//            case RETRACTING: slowPower = 0.3; fastPower = 0.8; normalPower = 0.75; break;
//            default: slowPower = 0.3; fastPower = 0.8; normalPower = 0.5; break;
//        }
//
//        if(slow && fast) setDrivePower(normalPower);
//        else if(slow) setDrivePower(slowPower);
//        else if(fast) setDrivePower(fastPower);
//        else setDrivePower(normalPower);
    }

    public void resetHeading(double heading) {
        fusedLocalizer.resetHeading(heading);
        Log.i("Drivetrain", "Reset heading to " + heading);
    }

    // set initial pose from auto
    public void initializePose() {
        Log.i("Drivetrain", "Initialized pose to: " + Globals.startPose);
        setPoseEstimate(Globals.startPose);
        resetHeading(Globals.startPose.getHeading());
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        this.targetHeading = targetPose.getHeading();
    }

    public boolean isAtTargetPose() {
        boolean translationAtTarget = getPoseEstimate().vec().distTo(targetPose.vec()) < TRANSLATION_AT_POSE_TOLERANCE;

        double headingError = getExternalHeading() - targetHeading;
        if(headingError < -Math.PI) headingError += 2*Math.PI;
        else if(headingError > Math.PI) headingError -= 2 * Math.PI;

//        Log.i("Drivetrain", "Heading error: " + headingError);
        boolean headingAtTarget = Math.abs(headingError) < HEADING_AT_POSE_TOLERANCE;

        boolean velocityAtTarget = getPoseVelocity().vec().norm() < 8.0;
        return translationAtTarget && headingAtTarget && velocityAtTarget;
    }

    public boolean inRange(Pose2d targetPose, double translationTolerance) {
        boolean translationAtTarget = getPoseEstimate().vec().distTo(targetPose.vec()) < translationTolerance;

        double headingError = getExternalHeading() - targetPose.getHeading();
        if(headingError < -Math.PI) headingError += 2*Math.PI;
        else if(headingError > Math.PI) headingError -= 2 * Math.PI;

        boolean headingAtTarget = Math.abs(headingError) < HEADING_AT_POSE_TOLERANCE;

        return translationAtTarget && headingAtTarget;
    }

    public void updateAprilTags(AprilTagProcessor processor) {
        try {
            fusedLocalizer.updateAprilTags(processor);
        } catch (Exception e) {
            return;
        }
    }

    public void updateAprilTags() {
        try {
//            fusedLocalizer.updateAprilTags(Robot.getInstance().cvMaster.tagDetector);
        } catch (Exception e) {
            return;
        }
    }

    public double getHeading() {return getExternalHeading();}

    public boolean isStopped() {
        return getPoseVelocity().vec().norm() < 0.1 && Math.abs(getExternalHeadingVelocity()) < 0.1;
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return super.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    idle();
                }); // idle the drivetrain before building a trajectory
    }

    public void ftcDashDrawPose() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay()
                        .setStroke("#1d38cf");
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void logPose() {
        Log.i("Drivetrain", "logged pose: " + getPoseEstimate());
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("drive power", drivePower);
        telemetry.addData("field centric", fieldCentric);
        telemetry.addData("target heading", targetHeading);
        telemetry.addData("DRIVETRAIN STATE:", drivetrainState);
        telemetry.addData("pose", getPoseEstimate());
        telemetry.addData("velocity", getPoseVelocity());
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("dt", dt);
        telemetry.addData("last drive vector", lastDriveVector);
        telemetry.addData("last drive vector magnitude", lastDriveVector.norm());
    }
}
