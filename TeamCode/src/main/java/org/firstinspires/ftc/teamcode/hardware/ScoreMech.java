package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class ScoreMech extends Mechanism {

    public PivotSlide pivotSlide = new PivotSlide();
    WristClaw wristClaw = new WristClaw();
    ElapsedTime dropTimer = new ElapsedTime();

    ElapsedTime pivotTimer = new ElapsedTime();
    ElapsedTime slideTimer = new ElapsedTime();

    public enum scoreStates {
        PIVOT_PREP,
        SLIDE_PREP,
        READY, //ready position, before we drop a cone to score
        GRAB, //state of about to intake a cone
        SCORE_DELAY, IDLE,
    }

    public scoreStates queuedState;
    public scoreStates currentState;

    public static double dropDelay = 300;
    public static double slideTarget = 0;
    public static double pivotTarget = 0;

    public static double slideDelay = 600;
    public static double pivotDelay = 600;

    public static boolean wristOverride = false;
    public static boolean cycling = false;
// daniel was here
    @Override
    public void init(HardwareMap hwMap) {
        pivotSlide.init(hwMap);
        wristClaw.init(hwMap, -PivotSlide.PIVOT_START_ANGLE);
        dropTimer.reset();
        slideTimer.reset();
        pivotTimer.reset();
        currentState = scoreStates.IDLE;
    }

    public void loop() {
        switch (currentState) {
            case PIVOT_PREP:
                pivotSlide.setPivotAngle(pivotTarget); //setting the angle sets isReached to false
                if(pivotSlide.pivot_isReached) { //isReached becomes true when the error is within a certain defined error bound
                    pivotSlide.resetSlide();
                    currentState = queuedState; //set FSM state to the state we want, either IDLE or GRAB
                }
                break;
            case SLIDE_PREP:
                wristOverride = false;
                pivotSlide.setSlideTarget(0); //we only ever come to this state from GRAB or IDLE, and we should retract before we pivot
                if(queuedState == scoreStates.GRAB) { //if you don't turn the pivot a lil while retracting after scoring you literally bang into the pole gg
                    pivotSlide.setPivotAngle(PivotSlide.INTERMEDIATE_ANGLE);
                }
                if(pivotSlide.slide_isReached) { //set to false when slide target is set, true when error is within bound
                    currentState = scoreStates.PIVOT_PREP; //time to turn the pivot
                }
                break;
            case READY:
                pivotSlide.setSlideTarget(slideTarget); //eventually will become high/mid/low pole positions
                break;
            case GRAB:
                pivotSlide.setSlideTarget(PivotSlide.MAX); //intake cone from as far away as possible
                break;
            case SCORE_DELAY:
                if(dropTimer.milliseconds() >= dropDelay) {
                    if(cycling) {
                        grab();
                    }else {
                        idle();
                    }
                }
                break;
            case IDLE:
                pivotSlide.setSlideTarget(0);
                break;
        }
        if(currentState == scoreStates.READY) {
            wristClaw.wristPitch(-pivotSlide.getPivotAngle()); //wrist pitch for easier cone scoring
        }else if(wristOverride){
            wristClaw.coneStackPitch(); //conestack
        }else {
            wristClaw.setWristAngle(-pivotSlide.getPivotAngle()); //wireless 4 bar to keep wrist parallel to ground
        }
        pivotSlide.update(); //update method updates both pivot and slides
    }

    public void update() {
        switch(currentState) {
            case SLIDE_PREP:
                wristOverride = false;
                pivotSlide.setSlideTarget(0);
                slideTimer.reset();
                if(queuedState == scoreStates.GRAB) {
                    pivotSlide.setPivotAngle(PivotSlide.INTERMEDIATE_ANGLE);
                }
                currentState = scoreStates.PIVOT_PREP;
                break;
            case PIVOT_PREP:
                if(slideTimer.milliseconds() >= slideDelay) {
                    pivotSlide.resetSlide();
                    pivotSlide.setPivotAngle(pivotTarget);
                    pivotTimer.reset();
                    currentState = queuedState;
                }
                break;
            case READY:
                if(pivotTimer.milliseconds() >= pivotDelay) {
                    pivotSlide.setSlideTarget(slideTarget); //eventually will become high/mid/low pole positions
                }
                break;
            case GRAB:
                if(pivotTimer.milliseconds() >= pivotDelay) {
                    pivotSlide.setSlideTarget(PivotSlide.MAX); //intake cone from as far away as possible
                }
                break;
            case IDLE:
                pivotSlide.setSlideTarget(0);
                break;
        }
        if(currentState == scoreStates.READY) {
            wristClaw.wristPitch(-pivotSlide.getPivotAngle());
        }else {
            wristClaw.setWristAngle(-pivotSlide.getPivotAngle());
        }
        pivotSlide.update();
    }

    public void ready(double position) {
        slideTarget = position;
        if(currentState != scoreStates.READY) {
            wristClaw.closeClaw();
            pivotTarget = PivotSlide.TELEOP_SCORE_ANGLE;
            queuedState = scoreStates.READY;
            currentState = scoreStates.SLIDE_PREP;
        }
    }

    public void grab() {
        if(currentState != scoreStates.GRAB) {
            pivotTarget = PivotSlide.GRAB_ANGLE;
            queuedState = scoreStates.GRAB;
            currentState = scoreStates.SLIDE_PREP;
        }else {
            idle();
        }
    }

    public void idle() {
        pivotTarget = PivotSlide.PIVOT_START_ANGLE;
        queuedState = scoreStates.IDLE;
        currentState = scoreStates.SLIDE_PREP;
    }



    public void toggleClaw() {
        if(currentState == scoreStates.GRAB || currentState == scoreStates.IDLE) {
            wristClaw.toggleClaw();
        }
        if(currentState == scoreStates.READY) {
            wristClaw.toggleClaw();
            dropTimer.reset();
            currentState = scoreStates.SCORE_DELAY;
        }
    }


}
