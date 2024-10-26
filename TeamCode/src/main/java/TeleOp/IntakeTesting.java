package TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "IntakeTesting")
@Config
public class IntakeTesting extends LinearOpMode {

  /*  DriveTrain driveTrain;
    Intake intake;

    ServoImplEx servotest;

    PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500,2500); */

    DcMotorEx motor;

    ServoImplEx servoL;
    ServoImplEx servoR;

    public static double pow = 0.3;
    public static double armPos = 0.25;
    private void initialize() {

        /*driveTrain = new DriveTrain(hardwareMap,telemetry,gamepad1);
        servotest = (ServoImplEx)hardwareMap.servo.get("servoTest");
        servotest.setPwmRange(pwmRange);
        //intake  = new Intake(hardwareMap,telemetry,gamepad1); */
        //motor = (DcMotorEx) hardwareMap.dcMotor.get("motor");
        servoL = (ServoImplEx)hardwareMap.servo.get("intakeL");
        servoR = (ServoImplEx)hardwareMap.servo.get("intakeR");
        servoR.setDirection(Servo.Direction.REVERSE);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            initialize();
        }
        while (opModeIsActive()){
            /*
            if(gamepad1.right_bumper){
                pow=0.2;
            }
            if(gamepad1.left_bumper){
                pow=-0.2;
            }
            if(gamepad1.b){
                pow = 0;
                servoL.setPwmDisable();
                servoR.setPwmDisable();
            } */


            servoR.setPosition(armPos);
            servoL.setPosition(armPos);

            //motor.setPower(pow);
            telemetry.addData("motorpow",pow);
            telemetry.addData("servoRpos",servoR.getPosition());
            telemetry.addData("servoLpos",servoL.getPosition());
        }
    }
}
