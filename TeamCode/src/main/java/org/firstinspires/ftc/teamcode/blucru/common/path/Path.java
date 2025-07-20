package org.firstinspires.ftc.teamcode.blucru.common.path;

public interface Path {
    // interface for drivetrain paths
    /*
        TODO: put all PID pathing into a package,
            and create another package for spline pathing.
            Look at boxtube spline for spline pathing

            I noticed pid paths are good for open driving,
            but time dependent spline pathing may be better for tighter spaces

            generate a spline, and pid to the current spline position
     */
    Path start();

    void run();

    void cancel();

    boolean isDone();
}
