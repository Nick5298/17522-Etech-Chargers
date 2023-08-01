package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.SplineInterpolator;

import java.util.Arrays;
import java.util.List;
@Config
public class PivotSlide extends Mechanism {
    //Parts
    public DcMotorEx slide, pivot;
    ElapsedTime slideTimer = new ElapsedTime();
    ElapsedTime pivotTimer = new ElapsedTime();

    //Current Power/Pos for parts for hardware write optimization
    public double pivotPower;
    public double slidePower;
    public double slide_currentPos;
    public double pivot_currentPos;

    //Physical Constants
    public static double SLIDE_TICKS_PER_REV = 145.1;
    public static double PIVOT_TICKS_PER_REV = 2786.2;
    public static double SLIDE_PULLEY_RADIUS = 4.40945; //in
    public static double PIVOT_START_ANGLE = -47.8; //deg

    //Slide positions
    public static double MAX = 770;
    public static double HIGH = 760;
    public static double LOW = 0;
    public static double MID = 335;

    //Pivot positions
    public static double GRAB_ANGLE = -12;
    public static double TELEOP_SCORE_ANGLE = 135;
    public static double AUTO_SCORE_ANGLE = 115;
    public static double INTERMEDIATE_ANGLE = 70;

    //PID Stuff //kG interpolate it based on length
    public static double slide_kP = 0.005;
    public static double slide_kD = 0;
    public static double slide_kG = 0.55; //high kG
    public static double slide_kG_2 = -0.15; //helps slides come back if angle < 0
    public static double slide_target = 0;
    public static double slide_lastError = 0;
    public static double slide_errorBound = 10;
    public static double slide_retractionMultiplier = 0.5;
    public boolean slide_isReached = false;

    public static double pivot_kP = 0.00275;
    public static double pivot_kD = 0.08;
    public static double pivot_target = 0;
    public static double pivot_lastError = 0;
    public static double pivot_kG = 0.05;
    public static double pivot_kG_slope = 0.0004; //i love lerp
    public static double pivot_kS = 0.2;
    public static double pivot_errorBound = 10;
    public boolean pivot_isReached = false;
    SplineInterpolator pivotAntiGrav;


    @Override
    public void init(HardwareMap hwMap) {
        slide = hwMap.get(DcMotorEx.class, "slide");
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        reset();
        slide.setPower(-0.1);
        slideTimer.reset();
        pivotTimer.reset();
    }

    public void setSlideTarget(double pos) {
        if(pos != slide_target) {
            slide_target = pos;
            slide_isReached = false;
        }
    }

    public void setPivotTarget(double pos) {
        if(pos != pivot_target) {
            pivot_target = pos;
            pivot_isReached = false;
        }
    }

    public void setPivotAngle(double angle) {
        setPivotTarget(angleToPos(angle));
    }

    public void update() {
        slideUpdate();
        pivotUpdate();
    }

    public void slideUpdate() {
        double time = slideTimer.milliseconds();
        slide_currentPos = slide.getCurrentPosition();
        double error = slide_target - slide_currentPos;
        double proportional = error * slide_kP;
        double derivative = (error - slide_lastError) / time * slide_kD;
        double pivotAngle = Math.toRadians(getPivotAngle());
        double antigravity = Math.sin(pivotAngle) * slide_kG * slide_target / MAX;
        double power = proportional + derivative;
        if(pivotAngle > 0) {
            power += antigravity;
        }else if(slide_target != 0) {
            power -= slide_kG_2 * Math.signum(error);
        }
        if(Math.abs(error) < slide_errorBound) {
            if(pivotAngle > 0) {
                power = antigravity;
            }
            if(Math.abs(error) < slide_errorBound + 10) {
                slide_isReached = true;
            }
        }
        if(slide_target == 0) {
            if(pivotAngle > Math.toRadians(70)) {
                power *= (slide_retractionMultiplier);
            }else {
                power += slide_kG_2;
            }
        }
        slide_lastError = error;
        slideTimer.reset();
        setSlidePower(power);
    }

    public void pivotUpdate() {
        double time = pivotTimer.milliseconds();
        pivot_currentPos = pivot.getCurrentPosition();
        double error = pivot_target - pivot_currentPos;
        double proportional = error * pivot_kP;
        double derivative = (error - pivot_lastError) / time * pivot_kD;
        double antigravity = Math.cos(Math.toRadians(getPivotAngle())) * (pivot_kG + pivot_kG_slope * slide_target);
        double power = Range.clip(
                proportional + derivative + antigravity + pivot_kS*Math.signum(error) * slide_target / MAX,
                -1, 1);
        if(Math.abs(error) < pivot_errorBound) {
            power = antigravity;
            if(Math.abs(error) < pivot_errorBound + 10) {
                pivot_isReached = true;
            }
        }
        pivot_lastError = error;
        pivotTimer.reset();
        setPivotPower(power);
    }

    public boolean isSlideReached() {
        double error = slide_target - slide_currentPos;
        return Math.abs(error) <= slide_errorBound+5;
    }

    public boolean isPivotReached() {
        double error = pivot_target - pivot_currentPos;
        return Math.abs(error) <= pivot_errorBound+10;
    }

    public void setSlidePower(double power) {
        if(slidePower != power) {
            slide.setPower(power);
            slidePower = power;
        }
    }

    public void setPivotPower(double power) {
        if(pivotPower != power) {
            pivot.setPower(power);
            pivotPower = power;
        }
    }

    public void reset() {
        resetSlide();
        resetPivot();
    }

    public void resetSlide() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_target = 0;
        slide_lastError = 0;
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetPivot() {
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_target = 0;
        pivot_lastError = 0;
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //arm angle in degrees
    public double getPivotAngle() {
        return pivot.getCurrentPosition() / PIVOT_TICKS_PER_REV * 360 + PIVOT_START_ANGLE;
    }

    public double getSlideTicks() {
        return slide.getCurrentPosition();
    }

    //make interpolation of arm anti grav
    public void initInterpolator(Double[] x, Double[] y) {
        List<Double> pivotKGvalues = Arrays.asList(y);
        List<Double> slidePositions = Arrays.asList(x);
        pivotAntiGrav = SplineInterpolator.createMonotoneCubicSpline(slidePositions, pivotKGvalues);
    }

    //slides extension in inches
    public double ticksToInches(double ticks) {
        return SLIDE_PULLEY_RADIUS * Math.PI * ticks / SLIDE_TICKS_PER_REV;
    }


    public double angleToPos(double angle) {
        return PIVOT_TICKS_PER_REV * (angle - PIVOT_START_ANGLE) / 360;
    }
}
