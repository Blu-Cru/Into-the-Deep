package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.pto_servo;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public class LeftPTOServo extends BluServo implements PTOServo {
    public LeftPTOServo () {
        super("ptoleft");
    }

    @Override
    public void init() {
        super.init();
        disengage();
    }

    @Override
    public void engage() {
        setPosition(0.15);
    }

    @Override
    public void disengage() {
        setPosition(0.32);
    }
}
