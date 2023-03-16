package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.function.Max;
import org.apache.commons.math3.analysis.function.Min;
@Config
public class WristClaw extends Mechanism {
    //Parts
    Servo claw, wrist;

    //Physical Constants
    public static double MAX_WRIST_ANGLE = 90; //deg
    public static double MIN_WRIST_ANGLE = -135;
    public static double MAX_CLAW_POSITION = 0.75;
    public static double MIN_CLAW_POSITION = 0.25;
    public static double WRIST_PITCH = 10; //deg

    //Current Claw/Wrist position for hardware write optimization
    public double clawPos;
    public double wristPos;

    //wrist positions
    public static double INIT_ANGLE = 0;

    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        wrist = hwMap.get(Servo.class, "wrist");
    }

    public void init(HardwareMap hwMap, double angle) {
        init(hwMap);
        setWristAngle(angle);
        openClaw();
    }

    public void toggleClaw() {
        if(clawPos == MAX_CLAW_POSITION) {
            closeClaw();
        }
        if(clawPos == MIN_CLAW_POSITION) {
            openClaw();
        }
    }

    public void openClaw() {
        setClawPosition(MAX_CLAW_POSITION);
    }

    public void closeClaw() {
        setClawPosition(MIN_CLAW_POSITION);
    }

    public void wristPitch(double angle) {
        setWristAngle(angle - WRIST_PITCH);
    }

    //degrees only
    public void setWristAngle(double angle) {
        angle = Range.clip(angle, MIN_WRIST_ANGLE, MAX_WRIST_ANGLE);
        angle = Range.scale(angle, MIN_WRIST_ANGLE, MAX_WRIST_ANGLE, 0, 0.88);
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
