package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class OrthogonalDistanceSensors implements BluSubsystem, Subsystem {
    Rev2mDistanceSensor leftSensor, rightSensor;
    public boolean reading;
    double leftDistance, rightDistance;

    public OrthogonalDistanceSensors() {
        leftSensor = Globals.hwMap.get(Rev2mDistanceSensor.class, "distleft");
        rightSensor = Globals.hwMap.get(Rev2mDistanceSensor.class, "distright");
    }

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

    public Pose2d getPos(double robotHeading){
        try{
            return OrthogonalDistanceSensorsPoseEstimation.getRobotPose(leftDistance, rightDistance, robotHeading);
        } catch (Exception e) {
            Log.e("OrthogonalDistanceSensors", "Either reading same wall or robot out of bounds");
        }
        return new Pose2d();
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

        static final double FIELD_WALL_OFFSET = 72.0,
         ANGLE_SINGLE_WALL_TOLERANCE = 0.3;// inches from center of field

        static final double[] rangeQ1 = {Math.PI , Math.PI * 3/2.0},
        rangeQ2 = {Math.PI*3/2.0, Math.PI * 2},
        rangeQ3 = {0, Math.PI / 2},
        rangeQ4 = {Math.PI/2, Math.PI};
        static final double[][] quadrantRanges = {rangeQ1, rangeQ2, rangeQ3, rangeQ4};

        static final double[][] quadrantToCorner = {{72,72}, {-72,72}, {-72,-72}, {72,-72}};

        static final double inFieldBoundValue = 63;
        public static Pose2d getRobotPose (double leftDistance, double rightDistance, double robotHeading) throws Exception {

            Vector2d dL = LEFT_SENSOR_ROBOT_POSE.vec().plus(Vector2d.polar(leftDistance, LEFT_SENSOR_ROBOT_POSE.getHeading()));
            Vector2d dR = RIGHT_SENSOR_ROBOT_POSE.vec().plus(Vector2d.polar(rightDistance, RIGHT_SENSOR_ROBOT_POSE.getHeading()));

            Vector2d fieldCentricdL = dL.rotated(robotHeading);
            Vector2d fieldCentricdR = dR.rotated(robotHeading);


            if (isSameWall(fieldCentricdL, fieldCentricdR)) {
                throw new Exception("Reading same wall");
            }

            int quadrant = 0;

            for (int i = 0; i < quadrantRanges.length; i++){
                if (robotHeading > quadrantRanges[i][0] && robotHeading <= quadrantRanges[i][1]){
                    quadrant = i;
                    break;
                }
            }

            double[] coords = quadrantToCorner[quadrant];


            double xVal;
            double yVal;

            if ((quadrant+1) % 2 == 1){
                xVal = fieldCentricdL.getX();
                yVal = fieldCentricdR.getY();
            } else {
                xVal = fieldCentricdR .getX();
                yVal = fieldCentricdL.getY();
            }

            double poseXVal = coords[0] + xVal;
            double poseYVal = coords[1] + yVal;

            if (Math.abs(poseXVal) <= inFieldBoundValue && Math.abs(poseYVal) <= inFieldBoundValue){
                throw new Exception("Robot position out of the field");
            }
            return new Pose2d(poseXVal, poseYVal, robotHeading);

        }

        public static boolean isSameWall(Vector2d fieldCentricdL, Vector2d fieldCentricdR){

            Vector2d differenceVector = fieldCentricdL.minus(fieldCentricdR);
            double angle = differenceVector.angle();

            double mod = angle % Math.PI/2;

            return mod < ANGLE_SINGLE_WALL_TOLERANCE || mod > Math.PI/2 - ANGLE_SINGLE_WALL_TOLERANCE;


        }



    }
}
