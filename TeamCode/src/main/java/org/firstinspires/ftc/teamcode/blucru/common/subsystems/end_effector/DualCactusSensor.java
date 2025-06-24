package org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Sample;

public class DualCactusSensor implements BluSubsystem {
    CactusSensor c1, c2;
    boolean lastValidSample, justValidSample;

    public DualCactusSensor() {
        c1 = new CactusSensor("spec cactus0", "spec cactus1");
        c2 = new CactusSensor("sample cactus0", "sample cactus1");
    }

    @Override
    public void init() {
        c1.init();
        c2.init();
    }

    @Override
    public void read() {
        c1.read();
        c2.read();

        if (validSample() && !lastValidSample)
            justValidSample = true;
        else
            justValidSample = false;
        lastValidSample = validSample();
    }

    @Override
    public void write() {
        c1.write();
        c2.write();
    }

    public boolean validSample() {
        return c1.validSample || c2.validSample;
    }

    public boolean empty() {
        return c1.isEmpty() && c2.isEmpty();
    }

    public boolean justValidSample() {
        return justValidSample;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Cactus Valid Sample", validSample());
        telemetry.addData("Cactus Empty", empty());
        c1.telemetry("c1", telemetry);
        c2.telemetry("c2", telemetry);
    }
}
