package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleParkFromSubIntakePath extends PIDPathBuilder {
    public SampleParkFromSubIntakePath() {
        super();
        this.setPower(0.9)
                .addMappedPoint(-42, -12, 20, 10)
                .addMappedPoint(-53, -53, 45);
    }
}
