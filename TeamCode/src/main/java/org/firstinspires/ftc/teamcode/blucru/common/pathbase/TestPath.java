package org.firstinspires.ftc.teamcode.blucru.common.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class TestPath extends PIDPathBuilder {
    public TestPath() {
        super();
        this.addMappedPoint(0, 20, 90)
                .waitMillis(300)
                .addMappedPoint(-7, 10, 120, 3)
                .addMappedPoint(-20, 0, 180, 3);
    }
}
