package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.PivotSlide;
import org.firstinspires.ftc.teamcode.hardware.WristClaw;
@TeleOp
@Config
public class WristTest extends LinearOpMode {

    WristClaw claw = new WristClaw();
    PivotSlide slide = new PivotSlide();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    public static double angle = 0;
    public static double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        slide.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                claw.openClaw();
            }
            if(gamepad1.b) {
                claw.closeClaw();
            }
            if(gamepad1.x) {
                claw.setWristPosition(angle);
            }
            if(gamepad1.y) {
                claw.setWristAngle(-slide.getPivotAngle());
            }
            tele.addData("pivot angle: ", slide.getPivotAngle());
            tele.update();
        }

    }
}
