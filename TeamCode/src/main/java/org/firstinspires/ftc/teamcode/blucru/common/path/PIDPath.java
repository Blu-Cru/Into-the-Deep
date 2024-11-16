package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;
import java.util.HashMap;

public class PIDPath implements Path {
    ArrayList<PathSegment> segmentList; // Path is made of a list of segments
    HashMap<Integer, ArrayList<Command>> commands; // each segment has a list of commands to run when started
    int segmentIndex;
    boolean pathDone;

    public PIDPath(ArrayList<PathSegment> segmentList, HashMap<Integer, ArrayList<Command>> commands) {
        this.segmentList = segmentList;
        this.commands = commands;
        segmentIndex = 0;
        pathDone = false;
    }

    @Override
    public Path start() {
        pathDone = false;
        segmentList.get(0).start();
        segmentIndex = 0;

        // run the commands associated with the first point
        try {
            for(Command c : commands.get(segmentIndex)) {
                c.schedule();
            }
        } catch (NullPointerException ignored) {}

        return this;
    }

    public void run() {
        Robot.getInstance().dt.pidTo(segmentList.get(segmentIndex).getPose());

        if(segmentList.get(segmentIndex).isDone() && !pathDone) {
            if(segmentIndex + 1 == segmentList.size()) {
                pathDone = true;
            } else {
                segmentIndex++;

                // run the commands associated with the next point
                try {
                    for(Command c : commands.get(segmentIndex)) {
                        c.schedule();
                    }
                } catch (NullPointerException ignored) {}

                segmentList.get(segmentIndex).start();
            }
        }
    }

    public void cancel() {
//        Robot.getInstance().dt.idle();
        pathDone = true;
    }

    public boolean failed() {
        return segmentList.get(segmentIndex).failed();
    }

    public boolean isDone() {
        return segmentIndex >= segmentList.size() || pathDone;
    }

    public void telemetry(Telemetry tele) {
        tele.addData("Path done: ", isDone());
        tele.addData("Path failed:", failed());
        tele.addData("Path index", segmentIndex);
    }
}
