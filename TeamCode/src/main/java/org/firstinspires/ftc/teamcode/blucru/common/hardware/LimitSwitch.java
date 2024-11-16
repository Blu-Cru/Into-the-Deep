package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class LimitSwitch {
    DigitalChannel digitalChannel;

    public LimitSwitch(DigitalChannel digitalChannel) {
        this.digitalChannel = digitalChannel;
    }

    public LimitSwitch(String name) {
        this(Globals.hwMap.get(DigitalChannel.class, name));
    }

    public boolean isPressed() {
        return !digitalChannel.getState();
    }

    public void telemetry() {
        Globals.tele.addData(digitalChannel.getDeviceName(), isPressed());
    }
}
