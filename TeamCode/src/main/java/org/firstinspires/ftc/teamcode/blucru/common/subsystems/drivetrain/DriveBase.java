package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.FusedLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class DriveBase implements BluSubsystem {
    public static Pose2d startPose = new Pose2d(0,0,Math.PI/2);
    FusedLocalizer localizer;

    BluMotor fl, fr, bl, br;
    BluMotor[] motors;

    public Pose2d pose, vel;
    public double heading, headingVel;
    public Vector2d xState, yState, headingState;

    public DriveBase() {
        localizer = new FusedLocalizer();

        fl = new BluMotor("fl", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        fr = new BluMotor("fr", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        bl = new BluMotor("bl", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        br = new BluMotor("br", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        xState = new Vector2d();
        yState = new Vector2d();
        headingState = new Vector2d();

        motors = new BluMotor[] {fl, fr, bl, br};
        localizer.setPoseEstimate(startPose);
    }

    @Override
    public void init() {
        localizer.setPoseEstimate(startPose);

        for (BluMotor motor : motors) {
            motor.init();
        }

        read();
    }

    @Override
    public void read() {
        localizer.update();

        pose = localizer.getPoseEstimate();
        vel = localizer.getPoseVelocity();
        heading = localizer.getHeading();
        headingVel = localizer.getHeadingVelocity();

        xState = new Vector2d(pose.getX(), vel.getX());
        yState = new Vector2d(pose.getY(), vel.getY());
        headingState = new Vector2d(heading, headingVel);

        for (BluMotor motor : motors) {
            motor.read();
        }
    }

    @Override
    public void write() {
        for (BluMotor motor : motors) {
            motor.write();
        }
    }

    public void drive(Pose2d pose) {
        pose = DriveKinematics.processStaticFriction(pose);

        double[] powers = DriveKinematics.getDrivePowers(pose);
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    public void drive(Vector2d translation, double rotation) {
        drive(new Pose2d(translation, rotation));
    }

    public void driveFieldCentric(Pose2d fieldDrivePose) {
        Pose2d robotDrivePose = new Pose2d(fieldDrivePose.vec().rotated(-localizer.getHeading()), fieldDrivePose.getHeading());
        drive(robotDrivePose);
    }

    public void driveFieldCentric(Pose2d driverInput, Alliance alliance) {
        if(alliance == Alliance.RED) {
            driveFieldCentric(driverInput);
            return;
        }

        Pose2d robotDrivePose = new Pose2d(driverInput.vec().rotated(-localizer.getHeading() + Math.PI), driverInput.getHeading());
        drive(robotDrivePose);
    }

    public void driveFieldCentric(Vector2d translation, double rotation) {
        driveFieldCentric(new Pose2d(translation, rotation));
    }

    public void driveFieldCentric(Vector2d translation, double rotation, Alliance alliance) {
        driveFieldCentric(new Pose2d(translation, rotation), alliance);
    }

    private Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose) {
        localizer.setPoseEstimate(pose);
    }

    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    public void setHeading(double heading, Alliance alliance) {
        if(alliance == Alliance.RED) localizer.setHeading(heading);
        else localizer.setHeading(Angle.norm(heading + Math.PI));
    }

    private double getHeading() {
        return localizer.getHeading();
    }

    private Pose2d getPoseVelocity() {
        return localizer.getPoseVelocity();
    }

    public void drawPose() {
        Globals.drawPose(getPoseEstimate());
    }

    public boolean updateAprilTags(AprilTagProcessor processor) {
        return localizer.updateAprilTags(processor);
    }

    public boolean updateAprilTags() {
        return localizer.updateAprilTags(Robot.getInstance().cvMaster.tagDetector);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        localizer.telemetry();
    }
}
