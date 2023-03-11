package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.function.Max;
import org.apache.commons.math3.analysis.function.Min;

public class WristClaw extends Mechanism {
    //Parts
    Servo claw, wrist;

    //Physical Constants
    public double MAX_WRIST_ANGLE = 270; //deg
    public double MAX_CLAW_POSITION = 1;
    public double MIN_CLAW_POSITION = 0;

    //Current Claw/Wrist position for hardware write optimization
    public double clawPos;
    public double wristPos;

    //wrist init position
    public static double INIT_ANGLE = 0;

    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        wrist = hwMap.get(Servo.class, "wrist");
        setWristAngle(INIT_ANGLE);
    }

    public void openClaw() {
        setClawPosition(MAX_CLAW_POSITION);
    }

    public void closeClaw() {
        setClawPosition(MIN_CLAW_POSITION);
    }

    //degrees only
    public void setWristAngle(double angle) {
        angle = Range.clip(angle, -MAX_WRIST_ANGLE, MAX_WRIST_ANGLE);
        angle = Range.scale(angle, -MAX_WRIST_ANGLE, MAX_WRIST_ANGLE, 0, 1);
        setWristPosition(angle);
    }


    //avoid redundant hardware writes
    public void setClawPosition(double position) {
        if(position != clawPos) {
            claw.setPosition(position);
            clawPos = position;
        }
    }

    public void setWristPosition(double position) {
        if(position != wristPos) {
            wrist.setPosition(position);
            wristPos = position;
        }
    }

}
