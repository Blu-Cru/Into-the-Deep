package org.firstinspires.ftc.teamcode.blucru.common.spline;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CubicHermiteSpline {
    double ax, bx, cx, dx; // coefficients of the x polynomial
    double ay, by, cy, dy; // coefficients of the y polynomial

    public CubicHermiteSpline(Vector2d start, Vector2d startVelocity, Vector2d end, Vector2d endVelocity) {
        ax = 2 * start.getX() + startVelocity.getX() - 2 * end.getX() + endVelocity.getX();
        bx = -3 * start.getX() - 2 * startVelocity.getX() + 3 * end.getX() - endVelocity.getX();
        cx = startVelocity.getX();
        dx = start.getX();

        ay = 2 * start.getY() + startVelocity.getY() - 2 * end.getY() + endVelocity.getY();
        by = -3 * start.getY() - 2 * startVelocity.getY() + 3 * end.getY() - endVelocity.getY();
        cy = startVelocity.getY();
        dy = start.getY();
    }

    public Vector2d getPoint(double t) {
        return new Vector2d(
                ax*t*t*t + bx*t*t + cx*t + dx,
                ay*t*t*t + by*t*t + cy*t + dy
        );
    }

    public Vector2d getVelocity(double t) {
        return new Vector2d(
                3*ax*t*t + 2*bx*t + cx,
                3*ay*t*t + 2*by*t + cy
        );
    }
}
