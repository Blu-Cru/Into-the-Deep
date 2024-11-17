package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import org.firstinspires.ftc.teamcode.blucru.common.util.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.FourSampleConfig;

// abstract class for auto config
public abstract class AutoConfig {
    // abstract methods to be implemented by the auto config
    public abstract void build();
    public abstract void start();
    public abstract void run();
    public abstract void telemetry();

    // this method returns the correct auto config based on the current side and auto type
    public static AutoConfig config() {
        if(Globals.autoType == AutoType.FOUR_SAMPLE) {
            return new FourSampleConfig();
        }
        return null;
    }
}
