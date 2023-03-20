package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    Robot bot = new Robot();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            bot.run(gamepad1);
            tele.addData("a: ", gamepad1.a);
            tele.addData("b: ", gamepad1.b);
            tele.addData("x: ", gamepad1.x);
            tele.addData("y: ", gamepad1.y);
            tele.addData("rb: ", gamepad1.right_bumper);
            tele.addData("lb: ", gamepad1.left_bumper);
            tele.addData("state: ", bot.scoring.currentState);
            tele.addData("pivot angle: ", bot.scoring.pivotSlide.getPivotAngle());
            tele.addData("pivot radians: ", Math.toRadians(bot.scoring.pivotSlide.getPivotAngle()));
            tele.addData("slide ticks: ", bot.scoring.pivotSlide.getSlideTicks());
            tele.update();
        }
    }
}
