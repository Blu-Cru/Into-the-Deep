package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.pto_servo;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public class RightPTOServo extends BluServo implements PTOServo {
    public RightPTOServo() {
        super("ptoright");
    }

    @Override
    public void init() {
        super.init();
        disengage();
    }

    @Override
    public void engage() {
        setPosition(0.3);
    }

    @Override
    public void disengage() {
        setPosition(0.15);
    }
}
