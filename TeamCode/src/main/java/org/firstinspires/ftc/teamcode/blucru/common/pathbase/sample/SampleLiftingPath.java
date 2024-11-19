package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BasketBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleLiftingPath extends PIDPathBuilder {
    public SampleLiftingPath() {
        super();
        this.setPower(0.3)
                .addMappedPoint(-50, -50, 45, 20)
                .schedule(new BasketBackHighCommand())
                .waitMillis(1000);
    }
}
