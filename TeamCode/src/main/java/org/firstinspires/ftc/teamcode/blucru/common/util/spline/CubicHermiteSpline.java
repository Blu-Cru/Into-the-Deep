package org.firstinspires.ftc.teamcode.blucru.common.util.spline;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class CubicHermiteSpline {
    double ax, bx, cx, dx; // coefficients of the x polynomial
    double ay, by, cy, dy; // coefficients of the y polynomial

    public CubicHermiteSpline(Pose2d start, Pose2d startVelocity, Pose2d end, Pose2d endVelocity) {
        ax = 2 * start.getX() + startVelocity.getX() - 2 * end.getX() + endVelocity.getX();
        bx = -3 * start.getX() - 2 * startVelocity.getX() + 3 * end.getX() - endVelocity.getX();
        cx = startVelocity.getX();
        dx = start.getX();

        ay = 2 * start.getY() + startVelocity.getY() - 2 * end.getY() + endVelocity.getY();
        by = -3 * start.getY() - 2 * startVelocity.getY() + 3 * end.getY() - endVelocity.getY();
        cy = startVelocity.getY();
        dy = start.getY();
    }
}
