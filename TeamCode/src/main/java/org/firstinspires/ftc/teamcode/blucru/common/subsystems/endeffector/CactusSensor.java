package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.IntakeState;

public class CactusSensor implements BluSubsystem {
    DigitalChannel channel0, channel1;
    boolean pin0, pin1;

    public CactusSensor() {
        channel0 = Globals.hwMap.get(DigitalChannel.class, "cactus0");
        channel1 = Globals.hwMap.get(DigitalChannel.class, "cactus1");
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {
        pin0 = channel0.getState();
        pin1 = channel1.getState();
    }

    @Override
    public void write() {

    }

    public IntakeState getState() {
        if(pin0 && pin1) return IntakeState.YELLOW;
        else if(pin0) return IntakeState.BLUE;
        else if(pin1) return IntakeState.RED;
        else return IntakeState.EMPTY;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Cactus Sensor", getState());
    }
}
