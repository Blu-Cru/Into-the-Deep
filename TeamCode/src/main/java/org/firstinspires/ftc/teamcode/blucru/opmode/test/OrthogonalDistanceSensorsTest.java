package org.firstinspires.ftc.teamcode.blucru.opmode.test;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.OrthogonalDistanceSensors;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group ="test")
public class OrthogonalDistanceSensorsTest extends BluLinearOpMode {
    public void initialize(){
        addOrthogonalDistanceSensors();
        addDrivetrain();
    }

    public void periodic(){
        orthogonalDistanceSensors.read();
        Pose2d pos = orthogonalDistanceSensors.getPos(dt.heading);
        telemetry.addData("X Pos:", pos.getX());
        telemetry.addData("Y Pos:", pos.getY());
        telemetry.addData("Heading", pos.getHeading());
    }
}
