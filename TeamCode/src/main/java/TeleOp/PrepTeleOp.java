package TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import Mechanisms.DriveTrain;
import Mechanisms.Intake;
import Mechanisms.Outake;

@TeleOp(name = "PrepTeleOp")
@Config
public class PrepTeleOp extends LinearOpMode {
    //DO NOT MODIFY THIS CLASS WHATSOEVER UNLESS AUTHORIZED-> MAKE A COPY
    DriveTrain driveTrain;
    Outake outake;
    Intake intake;

    DcMotorEx motor;
    Servo wrist;
    Servo armServoL;

    Servo clawU;
    public static double motorPow = 0.5;

    public static double restPow = 0;
    public static double clawPos = 0.5;

    public static int targetPos  = 100;

    public static double kf = 0.05;
    //0.72-deposit wrist
    //0.





    private void initialize() {
        driveTrain = new DriveTrain(hardwareMap,telemetry,gamepad1);
        outake = new Outake(hardwareMap,telemetry,gamepad2,driveTrain);
        intake = new Intake(hardwareMap,telemetry,gamepad1,outake);





    }




    @Override
    public void runOpMode() {
        initialize();

        while (opModeInInit()) {
            driveTrain.IMUSetup();
        }
        while (opModeIsActive()) {
            driveTrain.drive();
            intake.execute();
            outake.executeTeleOp();
            /*

            armServoL.setPosition(0.5);
            wrist.setPosition(clawPos);
            if(gamepad2.right_bumper){
                outake.slideR.setPower(motorPow);
                outake.slideL.setPower(motorPow);
            }
            else if(gamepad2.left_bumper){
                outake.slideL.setPower(-motorPow);
                outake.slideR.setPower(-motorPow);
            }
            else{
                outake.slideL.setPower(-restPow);
                outake.slideR.setPower(-restPow);
            }
            /*
            if(gamepad1.a)
                motor.setPower(motorPow);
            else if(gamepad1.b)
                motor.setPower(-motorPow);
            else
                motor.setPower(0);
            driveTrain.drive();

            wrist.setPosition(clawPos);
            outake.setKF(kf);
            //clawU.setPosition(clawPos);
        /*

            if(gamepad2.a)
                armServoL.setPosition(1);
            if(gamepad2.b)
                armServoL.setPosition(0.0);

            if(gamepad2.right_bumper){
                outake.slideR.setPower(motorPow);
                outake.slideL.setPower(motorPow);
            }
            else if(gamepad2.left_bumper){
                outake.slideL.setPower(-motorPow);
                outake.slideR.setPower(-motorPow);
            }
            else{
                outake.slideL.setPower(-restPow);
                outake.slideR.setPower(-restPow);
            }

            /*

             if(gamepad2.a)
                 outake.armServoL.setPosition(1);
             if(gamepad2.b)
                 outake.armServoL.setPosition(0.08);




            outake.setClawU(clawPos);
            outake.setClawB(clawPos);
            */
           // telemetry.addData("right trigger: ",gamepad1.right_trigger);
            telemetry.addData("motorPos",outake.slideL.getCurrentPosition());
            telemetry.update();
        }
    }
}
