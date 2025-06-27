package org.firstinspires.ftc.teamcode.blucru.common.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Sample;

import java.util.ArrayList;

public class SampleDetectionsList extends ArrayList<SampleDetection> {
    public SampleDetectionsList () {
        super();
    }

    public boolean add(SampleDetection detection){
        for (int i = 0; i < super.size(); i++){
            if (detection.distance < super.get(i).distance){
                super.add(i,detection);
                return true;
            }
        }
        super.add(detection);
        return false;
    }

    public Pose2d getBestSamplePose(Alliance alliance){

        //convert alliance to color
        Sample allianceColor;
        if (alliance == Alliance.BLUE){
            allianceColor = Sample.BLUE;
        } else {
            allianceColor = Sample.RED;
        }

        //find closest one
        for(int i = 0; i < super.size(); i++){
            if (super.get(i).color == allianceColor || super.get(i).color == Sample.YELLOW){
                return super.get(i).globalPose;
            }
        }
        return null;
    }

    public Pose2d getBestSpecPose(Alliance alliance){

        //convert alliance to color
        Sample allianceColor;
        if (alliance == Alliance.BLUE){
            allianceColor = Sample.BLUE;
        } else {
            allianceColor = Sample.RED;
        }

        //find closest one
        for(int i = 0; i < super.size(); i++){
            if (super.get(i).color == allianceColor){
                return super.get(i).globalPose;
            }
        }
        return null;
    }
}
