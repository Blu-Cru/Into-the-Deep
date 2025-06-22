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

    @Override
    public void onStart() {
        orthogonalDistanceSensors.reading = true;
    }

    public void periodic(){
        if (stickyG1.a){
            dt.setHeading(Math.PI/2);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Estimated pose", orthogonalDistanceSensors.getPos(dt.heading));
    }
}
