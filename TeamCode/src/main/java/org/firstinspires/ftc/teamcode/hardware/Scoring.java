package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Scoring extends Mechanism {

    PivotSlide pivotSlide = new PivotSlide();
    WristClaw wristClaw = new WristClaw();

    @Override
    public void init(HardwareMap hwMap) {
        pivotSlide.init(hwMap);
        wristClaw.init(hwMap);
    }


}
