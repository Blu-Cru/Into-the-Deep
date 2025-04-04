package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

public class PIDPathBuilder {
    ArrayList<PathSegment> segments;
    HashMap<Integer, ArrayList<Command>> commands; // commands to run before the point of the same index is reached
    ArrayList<Callback> callbacks;

    public PIDPathBuilder() {
        segments = new ArrayList<PathSegment>();
        commands = new HashMap<>();
        callbacks = new ArrayList<>();
    }

    public PIDPathBuilder addPoint(PIDPointSegment point) {
        segments.add(point);
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, double translationTolerance) {
        segments.add(new PIDPointSegment(pose, translationTolerance));
        return this;
    }

    public PIDPathBuilder addMappedPoint(double x, double y, double headingDeg, double translationTolerance) {
        this.addPoint(Globals.mapPose(x, y, headingDeg), translationTolerance);
        return this;
    }

    public PIDPathBuilder addMappedPoint(double x, double y, double headingDeg, boolean stopRequired) {
        this.addPoint(Globals.mapPose(x, y, headingDeg), stopRequired);
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose) {
        segments.add(new PIDPointSegment(pose));
        return this;
    }

    public PIDPathBuilder addMappedPoint(double x, double y, double headingDeg) {
        this.addPoint(Globals.mapPose(x, y, headingDeg));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, boolean stopRequired) {
        segments.add(new PIDPointSegment(pose, stopRequired));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, double translationTolerance, boolean stopRequired) {
        segments.add(new PIDPointSegment(pose, translationTolerance, stopRequired));
        return this;
    }

    public PIDPathBuilder addTurnToPoint(Vector2d drivePoint, Vector2d turnToPoint, double tolerance) {
        segments.add(new PIDTurnToPointSegment(drivePoint, turnToPoint, tolerance));
        return this;
    }

    public PIDPathBuilder addTurnToPoint(Vector2d drivePoint, Vector2d turnToPoint) {
        segments.add(new PIDTurnToPointSegment(drivePoint, turnToPoint));
        return this;
    }

    public PIDPathBuilder addMappedTurnToPoint(Vector2d drivePoint, Vector2d turnToPoint, double tolerance) {
        segments.add(new PIDTurnToPointSegment(Globals.mapPoint(drivePoint), Globals.mapPoint(turnToPoint), tolerance));
        return this;
    }

    public PIDPathBuilder addMappedTurnToPoint(Vector2d drivePoint, Vector2d turnToPoint) {
        segments.add(new PIDTurnToPointSegment(Globals.mapPoint(drivePoint), Globals.mapPoint(turnToPoint)));
        return this;
    }

    public PIDPathBuilder addSegment(PathSegment segment) {
        segments.add(segment);
        return this;
    }

    public PIDPathBuilder schedule(Command command) {
        commands.computeIfAbsent(segments.size(), k -> new ArrayList<Command>());
        commands.get(segments.size()).add(command);
        return this;
    }

    public PIDPathBuilder setPower(double power) {
        return schedule(new InstantCommand(() -> {
            Robot.getInstance().dt.setDrivePower(power);
        }));
    }

    public PIDPathBuilder callback(Callback callback) {
        while(callbacks.size() <= segments.size()) {
            callbacks.add(null);
        }
        callbacks.add(segments.size(), callback);
        return this;
    }

    public PIDPathBuilder waitMillis(double milliseconds) {
        segments.add(new WaitSegment(segments.get(segments.size() - 1), milliseconds));
        return this;
    }

    public PIDPath build() {
        while(callbacks.size() <= segments.size()) {
            callbacks.add(null);
        }
        return new PIDPath(segments, commands, callbacks);
    }

    public Path start() {
        return this.build().start();
    }
}
