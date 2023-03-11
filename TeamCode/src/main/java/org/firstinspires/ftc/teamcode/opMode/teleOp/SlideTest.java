package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.PivotSlide;

@TeleOp
@Config
public class SlideTest extends LinearOpMode {
    PivotSlide mech = new PivotSlide();
    public static double slideTarget = 0;
    public static double pivotTarget = 0;
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() throws InterruptedException {
        mech.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            mech.setSlideTarget(slideTarget);
            mech.setPivotTarget(pivotTarget);
            if(gamepad1.a) {
                mech.slideUpdate();
            }
            if(gamepad1.b) {
                mech.pivotUpdate();
            }
            if(gamepad2.y) {
                mech.resetSlide();
            }
            if(gamepad2.x) {
                mech.resetPivot();
            }

            tele.addData("pivot_angle: ", mech.getPivotAngle());
            tele.addData("slide_error: ", mech.slide_lastError);
            tele.addData("slide_pos: ", mech.slide.getCurrentPosition());
            tele.addData("pivot_error: ", mech.pivot_lastError);
            tele.addData("pivot_pos: ", mech.pivot.getCurrentPosition());
            tele.update();
        }
    }
}
