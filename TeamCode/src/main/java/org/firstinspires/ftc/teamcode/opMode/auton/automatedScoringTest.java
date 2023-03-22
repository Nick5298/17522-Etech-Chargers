package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.AutoMech;
import org.firstinspires.ftc.teamcode.hardware.PivotSlide;

@Autonomous
public class automatedScoringTest extends LinearOpMode {

    AutoMech auto = new AutoMech();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() throws InterruptedException {
        auto.init(hardwareMap);
        auto.ready(PivotSlide.HIGH);
        waitForStart();
        while(opModeIsActive()) {
            auto.loop();
            tele.addData("state: ", auto.currentState);
            tele.update();
        }
    }
}
