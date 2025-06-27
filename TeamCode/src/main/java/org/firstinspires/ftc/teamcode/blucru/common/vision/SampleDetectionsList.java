package org.firstinspires.ftc.teamcode.blucru.common.vision;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Sample;

import java.util.ArrayList;

public class SampleDetectionsList extends ArrayList<SampleDetection> {
    public SampleDetectionsList () {
        super();
    }

    public boolean add(SampleDetection detection){
        for (int i = 0; i < super.size(); i++){
            if (detection.getPenalty() < super.get(i).distance){
                super.add(i,detection);
                return true;
            }
        }
        super.add(detection);
        return false;
    }

    public Pose2d getBestSamplePose(){
        Alliance alliance = Globals.alliance;
        Log.i("SampleDetectionsList", "getting sample pose, alliance: " + alliance);

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
                Log.i("SampleDetectionsList", "Found sample with color " + super.get(i).color);
                return super.get(i).globalPose;
            }
        }
        return null;
    }

    public boolean hasSample() {
        //convert alliance to color
        Sample allianceColor;
        if (Globals.alliance == Alliance.BLUE){
            allianceColor = Sample.BLUE;
        } else {
            allianceColor = Sample.RED;
        }

        for (SampleDetection detection : this) {
            if(detection.color == allianceColor || detection.color == Sample.YELLOW) {
                return true;
            }
        }
        return false;
    }

    public boolean hasSpec() {
        //convert alliance to color
        Sample allianceColor;
        if (Globals.alliance == Alliance.BLUE){
            allianceColor = Sample.BLUE;
        } else {
            allianceColor = Sample.RED;
        }



        for (SampleDetection detection : this) {
            if(detection.color == allianceColor) {
                return true;
            }
        }
        return false;
    }

    public Pose2d getBestSpecPose(){
        Alliance alliance = Globals.alliance;

        //convert alliance to color
        Sample allianceColor;
        if (alliance == Alliance.BLUE){
            allianceColor = Sample.BLUE;
        } else {
            allianceColor = Sample.RED;
        }
        Log.i("SampleDetectionsList", "getting spec pose, alliance: " + Globals.alliance);
        //find best one
        for(int i = 0; i < super.size(); i++){
            if (super.get(i).color == allianceColor){
                Log.i("SampleDetectionsList", "Found sample with color " + super.get(i).color);
                return super.get(i).globalPose;
            }
        }
        return null;
    }

    public void telemetry() {
        Globals.tele.addLine("Sample Detections: ");
        for (SampleDetection detection : this) {
            Globals.tele.addLine(detection.color + " "
                    + detection.globalPose.toString()
                    + ", distance: " + detection.distance
                    + ", robot orientation: " + detection.robotDetectionOrientation
                    + ", penanty: " + detection.getPenalty());
        }
    }
}
