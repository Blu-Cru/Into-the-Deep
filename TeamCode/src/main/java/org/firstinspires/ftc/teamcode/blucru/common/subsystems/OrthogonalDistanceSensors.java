package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

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
}
