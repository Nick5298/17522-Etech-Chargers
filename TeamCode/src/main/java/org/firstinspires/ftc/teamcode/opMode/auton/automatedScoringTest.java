package org.firstinspires.ftc.teamcode.opMode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.AutoMech;
import org.firstinspires.ftc.teamcode.hardware.PivotSlide;

@Autonomous
public class automatedScoringTest extends LinearOpMode {

    AutoMech auto = new AutoMech();

    @Override
    public void runOpMode() throws InterruptedException {
        auto.init(hardwareMap);
        auto.ready(PivotSlide.HIGH);
        waitForStart();
        while(opModeIsActive()) {
            auto.loop();
        }
    }
}
