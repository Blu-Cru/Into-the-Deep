package org.firstinspires.ftc.teamcode.blucru.common.util;

public class Point3d {
    public double x, y, z;

    public Point3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }
}
