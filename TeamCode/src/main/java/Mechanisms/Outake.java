package Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Outake {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Gamepad gamepad2;
    //Change back to private later
    public ServoImplEx armServoL;
    public ServoImplEx armServoR;

    public Servo clawU;
    public Servo clawB;

    public Servo wrist;

    public Servo launcher;

    public DcMotorEx slideL;
    public DcMotorEx slideR;

    private double armPos  = 1;
    private double wristPos = 0.175;
    private double closePosB = 0.67;
    private double openPosB = 0.5;
    private double closePosU = 0.75;
    private double openPosU = 0.6;
    private static int height = 0;



    //PID
    private static ElapsedTime timer = new ElapsedTime();
    private static ElapsedTime slideTimer = new ElapsedTime();
    private static double lastError = 0;
    private static double integralSum =0;
    public  double Kp =0.0125;
    public  double Ki =0.0; //.00005
    public  double Kd =0.0;

    public static double Kf =0.05;

    private static double kl = 0.0;

    private static double pidOutput = 0;

    private static double lagError = 0.0;

    private static double lagOutput = 0;

    private int[] stripeLevels = {0,2099,2935};

    private int stripeIndex = 0;
    private static int targetPosition = 0;
    private static double currentPosition = 0;
    private static double manualPower = 0.5;


    private static double targetTolerance = 40;

    private static double lagTolerance = 40;

    private static double degreesToServo = 1.0/300;

    private static double bottomLevel = 0;
    //States
    private boolean isChanging;
    public enum OutakeStates{
        REST,
        EXTENSION,
        SCORE,
        RESET,

        RIGGING,

        MANUAL
    }

    enum SlidePreference{
        LEFT,
        RIGHT,
        BOTH
    }
    public OutakeStates currentOutakeState = OutakeStates.REST;
    private SlidePreference slidePreference = SlidePreference.BOTH;
    private DriveTrain driveTrain;
    //Constructor for Auto
    public Outake(HardwareMap hw, Telemetry tele){
        this.hardwareMap = hw;
        this.telemetry = tele;
        //Servo Init

        armServoL =  (ServoImplEx)hardwareMap.servo.get("armL");
        armServoR =  (ServoImplEx)hardwareMap.servo.get("armR");

        clawU = hardwareMap.servo.get("clawU");
        clawB = hardwareMap.servo.get("clawB");

        wrist = hardwareMap.servo.get("wrist");


        //change directions
        armServoR.setDirection(Servo.Direction.REVERSE);
        clawB.setDirection(Servo.Direction.REVERSE);

        //Motor Init
        slideL = (DcMotorEx) hardwareMap.dcMotor.get("slideL");
        slideR = (DcMotorEx) hardwareMap.dcMotor.get("slideR");

        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //change direction later
        //slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setDirection(DcMotorSimple.Direction.REVERSE);
        //disregard this if we want accurate pid ig
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Constructor for TeleOp
    public Outake(HardwareMap hw, Telemetry tele, Gamepad g1 , DriveTrain dt){
        this.hardwareMap = hw;
        this.telemetry = tele;
        this.gamepad2 = g1;
        this.driveTrain = dt;

        //Servo Init

        armServoL =  (ServoImplEx)hardwareMap.servo.get("armL");
        armServoR =  (ServoImplEx)hardwareMap.servo.get("armR");

        clawU = hardwareMap.servo.get("clawU");
        clawB = hardwareMap.servo.get("clawB");

        wrist = hardwareMap.servo.get("wrist");
        launcher = hardwareMap.servo.get("launcher");
        launcher.setPosition(0.5);

        //change directions
        armServoR.setDirection(Servo.Direction.REVERSE);
        clawB.setDirection(Servo.Direction.REVERSE);

        //Motor Init
        slideL = (DcMotorEx) hardwareMap.dcMotor.get("slideL");
        slideR = (DcMotorEx) hardwareMap.dcMotor.get("slideR");



        //change direction later
        //slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setDirection(DcMotorSimple.Direction.REVERSE);

        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //disregard this if we want accurate pid ig
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    //TeleOp Methods
    public void executeTeleOp(){
        //checks if DT is tipping
        //At any moment, you can set the target position(with dpad), and if the current state is either extension or manual, the slides would move
        //to target pos you set
        setTargetPos();
        if(armServoL.getPosition()<0.5){
            wrist.setPosition(0.7);
        }
        else if(armServoL.getPosition()>0.5){
            wrist.setPosition(0.72);
        }

        if(gamepad2.share){
            launcher.setPosition(1);
        }
        switch (currentOutakeState) {
            case REST:
                //when a is pressed, outake begins extension sequence
                if (gamepad2.a){
                    closeClawB();
                    closeClawU();
                    /*
                    clawU.setPosition(closePosU);
                    clawB.setPosition(closePosB);*/
                    currentOutakeState = OutakeStates.EXTENSION;
                }
                break;

            case EXTENSION:
                //when b is pressed again in extension state, outake resets
                if (gamepad2.dpad_down){
                    currentOutakeState = OutakeStates.RESET;
                    slideTimer.reset();
                }
                else if(gamepad2.options) currentOutakeState = OutakeStates.RIGGING;
                else {
                    //slides extend based off PID
                    //armServoR.setPosition(armPos);
                    //wrist.setPosition(wristPos);
                    //when x is pressed, clawB opens
                    if(gamepad2.y){
                        //clawB.setPosition(openPosB);
                        openClawU();
                    }
                    //when Y is pressed, clawU opens
                    if(gamepad2.x){
                        //clawU.setPosition(openPosU);
                        openClawB();
                    }
                    runSlidesPID();
                    if(slideL.getCurrentPosition()>400)
                        armServoL.setPosition(armPos);

                    //if bumper inputs are given, state switches to manual and pid is interrupted

                    if (gamepad2.right_bumper || gamepad2.left_bumper) {
                        currentOutakeState = OutakeStates.MANUAL;
                    }

                }
                break;

            case MANUAL:
                if (gamepad2.dpad_down){
                    currentOutakeState = OutakeStates.RESET;
                    slideTimer.reset();
                }
                else if(gamepad2.options) currentOutakeState = OutakeStates.RIGGING;
                else{

                    manual();

                    if(gamepad2.x){
                        clawB.setPosition(0.0);
                    }
                    if(gamepad2.y){
                        clawU.setPosition(0.0);
                    }
                    if(slideL.getCurrentPosition()>400)
                        armServoL.setPosition(armPos);

                //if the target position of the slides were changed during manual, the state is changed back to Extension so PID
                //starts again
                    if(isChanging){
                        isChanging = false;
                        currentOutakeState = OutakeStates.EXTENSION;
                    }
                //When down dpad is pressed, the currentState is changed to reset
                }
                break;

            case RIGGING:
                if(gamepad2.right_bumper){
                    currentOutakeState = OutakeStates.MANUAL;
                }
                else{
                    armServoL.setPosition(0.5);
                    if(1000-slideL.getCurrentPosition()<-40){
                        slideL.setPower(-0.5);
                        slideR.setPower(-0.5);
                    }
                    else{
                        slideL.setPower(-0.4);
                        slideR.setPower(-0.4);
                    }

                }

                break;


            case RESET:
                //the arm servos get reset
                //armServoR.setPosition(0.08);
                armServoL.setPosition(0.0);
                //wrist.setPosition(0.0);
                closeClawB();
                closeClawU();
                /*
                clawB.setPosition(closePosB);
                clawU.setPosition(closePosU);*/
                //While the slides are not at our target, the PID is being calculated
                //after the slides finally get to 0 position, the current state changes to rest

                if (Math.abs(targetPosition - currentPosition) > 10) {
                    //Only starts decreasing the slide height after the arm gets most of the way down
                   if(slideTimer.seconds()>=1){
                       runSlidesPID();
                   }
                }
                else {
                    currentOutakeState = OutakeStates.REST;
                }
                //If the bumpers interrupt the PID, the state switches to manual.
                /*
                if(gamepad2.right_bumper || gamepad2.left_bumper){
                    currentOutakeState = OutakeStates.MANUAL;
                } */
                break;


        }
        telemetry.addData("",slideL.getCurrentPosition());
    }
    public void executeAuto(){
        wrist.setPosition(0.7);
        switch(currentOutakeState){
            case EXTENSION:
                armServoL.setPosition(armPos);
                runSlidesPID();
                //armServoR.setPosition(armPos);
                //wrist.setPosition(wristPos);
                //when x is pressed, clawB open

            case RESET:
                armServoL.setPosition(0.08);
                //wrist.setPosition(0.0);
                closeClawB();
                closeClawU();
                /*
                clawB.setPosition(closePosB);
                clawU.setPosition(closePosU);*/
                //While the slides are not at our target, the PID is being calculated
                //after the slides finally get to 0 position, the current state changes to rest

                if (Math.abs(targetPosition - currentPosition) > 10) {
                    //Only starts decreasing the slide height after the arm gets most of the way down
                    if(armServoL.getPosition()<0.5){
                        runSlidesPID();
                    }
                }
                else {
                    currentOutakeState = OutakeStates.REST;
                }
                break;

            case REST:
                break;
        }
    }

    //needed for a check in intake
    public OutakeStates getOutakeState(){
        return currentOutakeState;
    }




    public void setTargetPos() {
        if(gamepad2.dpad_left){
            stripeIndex = 1;
        }
        if(gamepad2.dpad_right){
            stripeIndex = 2;
        }
        if(gamepad2.dpad_down){
            stripeIndex = 0;
        }

        //if the target position changes,
        if(Math.abs(targetPosition-stripeLevels[stripeIndex])>0){
            isChanging = true;
        }
        targetPosition = stripeLevels[stripeIndex];
        telemetry.addData("stripeIndex",stripeIndex);
        telemetry.addData("stripeLevels",stripeLevels[stripeIndex]);
        //telemetry.addData("targetPosition",targetPosition);
        telemetry.addData("isChanging",isChanging);
    }
    private void manual(){
        //slide control
        //might have to change it to velocity PID control instead
        if(gamepad2.right_bumper && slideL.getCurrentPosition()<3000){
            slideL.setPower(manualPower);
            slideR.setPower(manualPower);
        }
        else if(gamepad2.left_bumper && slideL.getCurrentPosition()>0){
            slideL.setPower(-manualPower);
            slideR.setPower(-manualPower);
        }
        else{
            slideL.setPower(Kf);
            slideR.setPower(Kf);
        }

    }

    //If not mechanically slaved and slides start lagging for some reason, it takes motor pos closest to target and focuses on that
    //in order to maintain the slight lag and not letting it get worse, lag tolerance is tuneable

    //PID calculations
    private void returnPower(){
        //evaluate();
        currentPosition = slideL.getCurrentPosition();
        double error = targetPosition - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error -lastError)/ timer.seconds();
        lastError = error;
        timer.reset();
        pidOutput = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        //lagOutput = kl*lagError + pidOutput;
        telemetry.addData("error: ",error);
    }
    //Appropriating and setting motor values
    public void runSlidesPID(){
        returnPower();
        if(Math.abs(targetPosition - currentPosition) > targetTolerance){
            slideL.setPower(pidOutput);
            slideR.setPower(pidOutput);

        }
        else{
            slideL.setPower(Kf);
            slideR.setPower(Kf);
        }

        telemetry.addData("currentPos: ",currentPosition);
        telemetry.addData("power: ", pidOutput);


    }

    //AUTO METHODS

    //claw controls
    public void closeClawB(){ clawB.setPosition(closePosB); }
    public void closeClawU(){ clawU.setPosition(closePosU); }
    public void openClawB(){
        clawB.setPosition(openPosB);
    }
    public void openClawU(){
        clawU.setPosition(openPosU);
    }

    public void setTargetPosition(int targetPosition){
        this.targetPosition = targetPosition;
    }

    public void setKF(double n ){
        Kf = n;
    }
    //Scoring automation


    //automated arm controls
    public void controlArm(){
        //double theta1 = armServoL.getPosition()*degreesToServo -
        armServoL.setPosition(180*degreesToServo);
        armServoR.setPosition(180*degreesToServo);
        if(armServoL.getPosition()>180*degreesToServo) {
            double theta1 = 22.5 * Math.cos((Math.PI / stripeLevels[0]) * (currentPosition - bottomLevel)) + 210; //cos function expressing
            if(currentPosition>stripeLevels[2])
                theta1 = 232.5;
            armServoL.setPosition(theta1 * degreesToServo);
            armServoR.setPosition(theta1 * degreesToServo);
        }
    }

    public void setClawU(double n){
        clawU.setPosition(n);
    }
    public void setClawB(double n){
        clawB.setPosition(n);
    }



}
