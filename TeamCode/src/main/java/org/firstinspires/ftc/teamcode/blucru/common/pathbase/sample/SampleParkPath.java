package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleParkPath extends PIDPathBuilder {
    public SampleParkPath() {
        super();
        this.setPower(0.2)
                .addMappedPoint(50, 40, 270, 4)
                .setPower(0.6)
                .addMappedPoint(48, 20, 315, 8)
                .addMappedPoint(26, 12, 0);
    }
}
