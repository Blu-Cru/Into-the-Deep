package org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class TeleSampleHighLiftPath extends PIDPathBuilder {
    public TeleSampleHighLiftPath() {
        super();
        this.setPower(0.8)
                .schedule(new PivotCommand(Math.PI/2))
                .addMappedPoint(-53, -53, 45, 5)
                .schedule(new SampleBackHighCommand())
                .waitMillis(200)
                .setPower(0.25)
                .addMappedPoint(-55.8, -55.8, 45);
    }
}
