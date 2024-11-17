package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleDepositPath extends PIDPathBuilder {
    public SampleDepositPath() {
        super();
        this.setPower(0.35)
                .addMappedPoint(55, 55, 225);

    }
}
