package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ScoreMech;
@TeleOp
public class ScoreTest extends LinearOpMode {

    ScoreMech mech = new ScoreMech();

    @Override
    public void runOpMode() throws InterruptedException {
        mech.init(hardwareMap);
    }
}
