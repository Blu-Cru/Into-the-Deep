package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class LimitSwitch implements BluSubsystem {
    DigitalChannel digitalChannel;

    public boolean state;
    boolean lastState;

    private LimitSwitch(DigitalChannel digitalChannel) {
        this.digitalChannel = digitalChannel;
    }

    public LimitSwitch(String name) {
        this(Globals.hwMap.get(DigitalChannel.class, name));
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {
        lastState = state;
        state = digitalChannel.getState();
    }

    @Override
    public void write() {

    }

    public boolean pressed() {
        return state;
    }

    public boolean changed() {
        return state != lastState;
    }

    public boolean justPressed() {
        return state && changed();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Limit Switch state:", state);
    }
}
