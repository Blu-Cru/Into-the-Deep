package org.firstinspires.ftc.teamcode.blucru.common.path;

public interface Path {
    // interface for both PID path following and RoadRunner path following
    Path start();

    void run();

    void cancel();

    boolean isDone();
}
