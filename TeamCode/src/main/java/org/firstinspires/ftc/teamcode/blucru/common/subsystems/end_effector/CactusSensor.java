package org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector;

import static org.firstinspires.ftc.teamcode.blucru.common.util.Globals.alliance;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Sample;

public class CactusSensor implements BluSubsystem {
    DigitalChannel channel0, channel1;
    boolean pin0, pin1;
    public boolean validSample, validSpecimen, justValidSample, justValidSpecimen;
    boolean lastValidSample, lastValidSpecimen;

    public CactusSensor(String config0, String config1) {
        channel0 = Globals.hwMap.get(DigitalChannel.class, config0);
        channel1 = Globals.hwMap.get(DigitalChannel.class, config1);
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {
        pin0 = channel0.getState();
        pin1 = channel1.getState();

        validSample = validSample();
        validSpecimen = validSpecimen();

        justValidSample = validSample && !lastValidSample;
        lastValidSample = validSample;

        justValidSpecimen = validSpecimen && !lastValidSpecimen;
        lastValidSpecimen = validSpecimen;
    }

    @Override
    public void write() {

    }

    public Sample getState() {
        if(pin0 && pin1) return Sample.YELLOW;
        else if(pin0) return Sample.BLUE;
        else if(pin1) return Sample.RED;
        else return Sample.EMPTY;
    }

    private boolean validSpecimen() {
        Sample state = getState();

        if(alliance == Alliance.RED) return state == Sample.RED;
        else return state == Sample.BLUE;
    }

    public boolean validSample() {
        Sample state = getState();

        if(state == Sample.YELLOW) return true;

        if(alliance == Alliance.RED) return state == Sample.RED;
        else return state == Sample.BLUE;
    }

    public boolean justValidSample () {
        return justValidSample;
    }

    public boolean empty() {
        return getState() == Sample.EMPTY;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Cactus Sensor", getState());
    }

    public void telemetry(String tag, Telemetry telemetry) {
        telemetry.addData(tag + "Cactus Sensor", getState());
    }
}
