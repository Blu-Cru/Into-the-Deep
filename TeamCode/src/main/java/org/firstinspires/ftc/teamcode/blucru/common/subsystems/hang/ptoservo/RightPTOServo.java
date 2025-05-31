package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.ptoservo;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public class RightPTOServo extends BluServo implements PTOServo {
    public RightPTOServo() {
        super("ptoright");
    }

    @Override
    public void engage() {
//        setPosition();
    }

    @Override
    public void disengage() {

    }
}
