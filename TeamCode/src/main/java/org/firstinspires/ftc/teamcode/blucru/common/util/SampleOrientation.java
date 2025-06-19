package org.firstinspires.ftc.teamcode.blucru.common.util;

public enum SampleOrientation {
    VERTICAL(Math.PI/2.0),
    HORIZONTAL(0.0),
    POSITIVE_45(Math.PI/4.0),
    NEGATIVE_45(-Math.PI/4.0);

    static SampleOrientation[] ORIENTATIONS = values();

    final double rad;

    SampleOrientation(double rad) {
        this.rad = rad;
    }

    public double angle() {
        return rad;
    }

    public SampleOrientation next() {
        int nextOrdinal = (this.ordinal() + 1) % ORIENTATIONS.length;
        return ORIENTATIONS[nextOrdinal];
    }

    public SampleOrientation prev() {
        int prevOrdinal = (this.ordinal() - 1) % ORIENTATIONS.length;
        return ORIENTATIONS[prevOrdinal];
    }
}