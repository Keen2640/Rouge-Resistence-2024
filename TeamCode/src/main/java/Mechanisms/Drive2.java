package Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drive2 {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Gamepad gamepad2;

    private Servo armServoL;
    private Servo armServoR;

    private Servo clawU;
    private Servo clawB;

    private Servo wrist;

    private DcMotorEx slideL;
    private DcMotorEx slideR;

    private double armPos  = 0.0;
    private double wristPos = 0.0;
    private double closeClaw = 0.0;
    private static int height = 0;



    //PID
    private static ElapsedTime timer = new ElapsedTime();

    private static double pidOutput;

    private static int lag;
    private static double lastError = 0;
    private static double integralSum =0;
    public static double Kp =0.0125;
    public static double Ki =0.0; //.00005
    public static double Kd =0.0;
    public static double Kf =0.0;

    public static double kl =0.0;

    public int[] stripeLevels = {1000,2000,3000};

    public int stripeIndex = 0;
    public static int targetPosition = 5;


    public static double manualPower = 0.5;

    public static double targetTolerance = 40;

    public static double lagTolerance = 40;


    //States

    private boolean isManual = true;
    private boolean isScoring;
    private boolean rightT;
    private boolean leftT;

    //DriveTrain info for anti-tip
    DriveTrain driveTrain;


    public Drive2(HardwareMap hw, Telemetry tele, Gamepad g2, DriveTrain driveTrain ){
        this.hardwareMap = hw;
        this.telemetry = tele;
        this.gamepad2 = g2;
        this.driveTrain =driveTrain;

        //Servo Init

        armServoL =  hardwareMap.servo.get("armServoL");
        armServoR =  hardwareMap.servo.get("armServoR");

        clawU = hardwareMap.servo.get("clawU");
        clawB = hardwareMap.servo.get("clawB");

        wrist = hardwareMap.servo.get("wrist");

        //change directions
        armServoL.setDirection(Servo.Direction.REVERSE);
        clawB.setDirection(Servo.Direction.REVERSE);

        //Motor Init
        slideL = (DcMotorEx) hardwareMap.dcMotor.get("slideL");
        slideR = (DcMotorEx) hardwareMap.dcMotor.get("slideR");

        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //change direction later
        slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setDirection(DcMotorSimple.Direction.REVERSE);
        //disregard this if we want accurate pid ig
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }
    public void drive2(){
       setTargetPos();
       if(gamepad2.y){
            clawU.setPosition(0.0);
       }
       if(gamepad2.x){
            clawB.setPosition(0.0);
       }
       if(gamepad2.a){
           isManual=false;
       }
       if(isManual){
           manual();
       }
       else {
            if (Math.abs(targetPosition - evaluate()) < targetTolerance && isScoring) {
                armServoR.setPosition(armPos);
                armServoL.setPosition(armPos);
                wrist.setPosition(wristPos);
                isScoring = false;
             }
            if (gamepad2.b) {
                armServoR.setPosition(0.0);
                armServoL.setPosition(0.0);
                wrist.setPosition(0.0);
                targetPosition = 0;
            }
            if(gamepad2.a) {
                 isManual = true;
              }
            //lag fixing and running PID
                fixLag();

       }



       //slides go down when both begins tilting, we basically hope the change in momentum from the slides going down causes bot to go down
       // with the drive train correcting itself at the same time
       if(driveTrain.isTipping()){
           armServoR.setPosition(0.0);
           armServoL.setPosition(0.0);
           wrist.setPosition(0.0);
           targetPosition = 0;
       }

    }

    private void manual(){
        //arm control
        if (gamepad2.right_trigger>0.1/*&& some value cap*/) {
            armServoR.setPosition(armServoR.getPosition()+0.05);
            armServoL.setPosition(armServoL.getPosition()+0.05);
        }
        else if (gamepad2.left_trigger>0.1) {
            armServoR.setPosition(armServoR.getPosition()-0.05);
            armServoL.setPosition(armServoL.getPosition()-0.05);
        }
        //wrist control
        if(gamepad2.dpad_right){
            wrist.setPosition(wrist.getPosition()+0.05);
        }
        else if (gamepad2.dpad_left){
            wrist.setPosition(wrist.getPosition()-0.05);
        }
        //slide control
        if(gamepad2.right_bumper){
            slideL.setPower(manualPower);
            slideR.setPower(manualPower);

        }
        else if(gamepad2.left_bumper){
            slideL.setPower(-manualPower);
            slideR.setPower(-manualPower);
        }
        else {
            slideL.setPower(Kf);
            slideR.setPower(Kf);
        }
    }
    //
    private void setTargetPos() {
        if(gamepad2.dpad_up){
            stripeIndex++;
        }
        if(gamepad2.dpad_down){
            stripeIndex--;
        }
        if(stripeIndex>stripeLevels.length-1){
            stripeIndex=stripeLevels.length-1;
        }
        if(stripeIndex<0){
            stripeIndex=0;
        }
        //if the target position changes,
        if(Math.abs(targetPosition-stripeLevels[stripeIndex])>0){
            isScoring = true;
        }
        targetPosition = stripeLevels[stripeIndex];

    }
    //PID stuffs, also when evaluate is called for PID calculation purposes
    // Kp, Kd, Ki->tuneable
    private void runPID(){
        double error = targetPosition - evaluate();
        integralSum += error * timer.seconds();
        double derivative = (error -lastError)/ timer.seconds();
        lastError = error;
        timer.reset();

        pidOutput = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    //returns slide that is closer to its target position and calculates lag
    private int evaluate(){
        //lagTolerance->tuneable
        //If the slides are lagging an amount less than our lag tolerance, then lag=0, and it returns the average position of the slides
        if(Math.abs(slideR.getCurrentPosition()-slideL.getCurrentPosition())<lagTolerance){
            lag = 0;
            return (slideL.getCurrentPosition()+slideR.getCurrentPosition())/2;
        }
        //
        else if(Math.abs(targetPosition-slideR.getCurrentPosition())>Math.abs(targetPosition-slideL.getCurrentPosition())){
            lag = slideL.getCurrentPosition()-slideR.getCurrentPosition();
            return slideL.getCurrentPosition();
        }
        else{
            lag = slideR.getCurrentPosition()-slideL.getCurrentPosition();
            return slideR.getCurrentPosition();
        }
    }

    //The fix lag method is available in order to combat any lagging from non mechanically slaved slide motors
    private void fixLag(){
        //running pid to get base motor output
        runPID();
        double lagCorrection = pidOutput+kl*lag;
        //re-sizing motor output,so when there is lag correction, the lagging motor pow =1 & the non lagging motor power<1
        //In order to properly correct the slides when the pidOutPut reaches max value of 1.
        if(Math.abs(lagCorrection)>1){
            lagCorrection/=lagCorrection;
            pidOutput/=lagCorrection;
        }
        //Diagnoses which motor is lagging and which motor is ahead, and applies the lag correction to the lagging motor
        if(0.5*(slideL.getCurrentPosition()+slideR.getCurrentPosition())==evaluate()){
            slideL.setPower(pidOutput);
            slideR.setPower(pidOutput);
        }
        else if(slideL.getCurrentPosition() == evaluate()){
            slideR.setPower(lagCorrection);
            slideL.setPower(pidOutput);
        }
        else{
            slideL.setPower(lagCorrection);
            slideR.setPower(pidOutput);
        }

    }


}
