package org.firstinspires.ftc.teamcode.blucru.common.spline;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class EndHermiteSpline extends CubicHermiteSpline{
    public EndHermiteSpline(Vector2d startPose, Vector2d startVelocity, Vector2d endPose) {
        super(startPose, startVelocity, endPose, new Vector2d(0, 0));
    }
}
