package org.firstinspires.ftc.teamcode.blucru.common.util;

public enum AutoType {
    FOUR_SAMPLE;

    // flip the starting side
    public AutoType flip() {
        if(this == FOUR_SAMPLE) {
            return FOUR_SAMPLE;
        } else {
            return FOUR_SAMPLE;
        }
    }
}
