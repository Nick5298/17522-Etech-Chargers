package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
public class Robot extends Mechanism {

    public ScoreMech scoring = new ScoreMech();
    public SampleMecanumDrive drive;

    public static double driveMultiplier = 0.5;
    public boolean isPressedA,
            isPressedB,
            isPressedX,
            isPressedY,
            isPressedRB,
            isPressedLB;

    @Override
    public void init(HardwareMap hwMap) {
        scoring.init(hwMap);
        drive = new SampleMecanumDrive(hwMap);
    }

    public void run(Gamepad pad) {
        move(pad);
        scoreControl(pad);
    }

    public void move(Gamepad pad) {
        if(scoring.currentState == ScoreMech.scoreStates.READY) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -pad.left_stick_y * driveMultiplier,
                            -pad.left_stick_x * driveMultiplier,
                            -pad.right_stick_x * driveMultiplier
                    )
            );
        }else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -pad.left_stick_y,
                            -pad.left_stick_x,
                            -pad.right_stick_x
                    )
            );
        }
    }

    public void scoreControl(Gamepad pad) {
        if(pad.a && !isPressedA) {
            scoring.ready(PivotSlide.LOW);
        }
        if(pad.b && !isPressedB) {

        }
        if(pad.x && !isPressedX) {
            scoring.ready(PivotSlide.MID);
        }
        if(pad.y && !isPressedY) {
            scoring.ready(PivotSlide.HIGH);
        }
        if(pad.left_bumper && !isPressedLB) {
            scoring.grab();
        }
        if(pad.right_bumper && !isPressedRB) {
            scoring.toggleClaw();
        }
        isPressedA = pad.a;
        isPressedB = pad.b;
        isPressedX = pad.x;
        isPressedY = pad.y;
        isPressedLB = pad.left_bumper;
        isPressedRB = pad.right_bumper;
        scoring.loop();
    }

}
