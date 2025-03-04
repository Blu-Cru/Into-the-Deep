package org.firstinspires.ftc.teamcode.blucru.common.path;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;
import java.util.HashMap;

public class PIDPath implements Path {
    ArrayList<PathSegment> segmentList; // Path is made of a list of segments
    HashMap<Integer, ArrayList<Command>> commands; // each segment has a list of commands to run when started
    ArrayList<Callback> callbacks;
    int segmentIndex;
    boolean pathDone;

    public PIDPath(ArrayList<PathSegment> segmentList, HashMap<Integer, ArrayList<Command>> commands) {
        this(segmentList, commands, new ArrayList<>(segmentList.size()));
    }

    public PIDPath(ArrayList<PathSegment> segmentList, HashMap<Integer, ArrayList<Command>> commands, ArrayList<Callback> callbacks) {
        this.segmentList = segmentList;
        this.commands = commands;
        this.callbacks = callbacks;
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
        } catch (NullPointerException ignored) {
            Log.e("PID Path", "error scheduling command");
        }

        // run the callbacks associated with the first point
        try {
            callbacks.get(segmentIndex).run();
        } catch (NullPointerException ignored) {
            Log.e("PID Path", "error scheduling command");
        }

        return this;
    }

    public void run() {
        if(isDone()) {
            return;
        }
        PathSegment currentSegment = segmentList.get(segmentIndex);
        currentSegment.run();
//        Robot.getInstance().dt.pidTo(currentSegment.getPose());

        if(currentSegment.isDone() && !pathDone) {
            segmentIndex++;

            try {
                for(Command c : commands.get(segmentIndex)) {
                    c.schedule();
                }
            } catch (NullPointerException ignored) {
                Log.e("PID Path", "error scheduling command");
            }

            // run the callbacks associated with the segment
            try {
                callbacks.get(segmentIndex).run();
            } catch (NullPointerException ignored) {
                Log.e("PID Path", "error running callback");
            }

            if(segmentIndex == segmentList.size()) {
                pathDone = true;
                return;
            }

            segmentList.get(segmentIndex).start();
        }
    }

    public void cancel() {
        Robot.getInstance().dt.idle();
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
