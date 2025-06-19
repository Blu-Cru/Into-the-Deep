package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OrthogonalDistanceSensors implements BluSubsystem, Subsystem {
    Rev2mDistanceSensor leftSensor, rightSensor;
    boolean reading;
    double leftDistance, rightDistance;

    @Override
    public void init() {
        reading = false;
    }

    @Override
    public void read() {
        if(reading) {
            leftDistance = leftSensor.getDistance(DistanceUnit.INCH);
            rightDistance = rightSensor.getDistance(DistanceUnit.INCH);
        }
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Distance (in)", leftDistance);
        telemetry.addData("Right Distance (in)", rightDistance);
    }

    /*
     class for localization using two distance sensors

     this is assuming the sensors are only measuring the field walls, nothing else is in the way
    */
    static class OrthogonalDistanceSensorsPoseEstimation {
        static final Pose2d
                // positions of sensors on the robot (x is forward, y is left)
                LEFT_SENSOR_ROBOT_POSE = new Pose2d(-7.0, 6.0, 3.0*Math.PI / 4.0),
                RIGHT_SENSOR_ROBOT_POSE = new Pose2d(-7.0, -6.0, -3.0*Math.PI / 4.0);

        static final double FIELD_WALL_OFFSET = 72.0; // inches from center of field

        public static Pose2d getRobotPose (double leftDistance, double rightDistance, double robotHeading) {
            return new Pose2d();
        }
    }
}
