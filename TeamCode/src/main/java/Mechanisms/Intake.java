package Mechanisms;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    private Gamepad gamepad1;

    private Servo intakeServoL;

    private Servo intakeServoR;

    private DcMotorEx rollerMotor;

    private double motorPow = 0.9;

    private double armPos = 0.0;


    private double bottomPos = 0.302;

    private int posIndex = 0;
    enum IntakeStates{
        INTAKE,
        EXTAKE,
        GROUND


    }
    private IntakeStates currentState = IntakeStates.GROUND;
    private Outake outake;

    public Intake(HardwareMap hw, Telemetry tele, Gamepad g1, Outake outake){
        this.hardwareMap = hw;
        this.telemetry = tele;
        this.gamepad1 = g1;
        this.outake = outake;
        //Servo Init
        ;
        //Reversing direction of right servo

        //Motor Init
        rollerMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        rollerMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void execute(){


        switch (currentState) {
            //When the roller is in the ground state, there is no movement of intake
            case GROUND:
                // the ground pixel intake level

               /* intakeServoL.setPosition(0.3);
                intakeServoL.setPosition(0.3); */
                rollerMotor.setPower(0.0);
                //When x is pressed and the outake is at rest, the state is switched to Moving
                if (gamepad1.x && (outake.getOutakeState()==Outake.OutakeStates.REST) ) {
                    currentState = IntakeStates.INTAKE;
                    outake.openClawB();
                    outake.openClawU();
                }
                if (gamepad1.b  && (outake.getOutakeState()==Outake.OutakeStates.REST) ) {
                    currentState = IntakeStates.EXTAKE;
                    outake.openClawB();
                    outake.openClawU();
                }
                break;

            case INTAKE:

                rollerMotor.setPower(motorPow);

                //if x is not held, then the state is switched back to ground
                if (!gamepad1.x) {

                    outake.closeClawB();
                    outake.closeClawU();
                    currentState = IntakeStates.GROUND;
                }
                break;

            case EXTAKE:
                rollerMotor.setPower(-motorPow);

                if (!gamepad1.b) {

                    outake.closeClawB();
                    outake.closeClawU();
                    currentState = IntakeStates.GROUND;
                }


            }


    }

    public void setMotorPow(double n){
        motorPow = n;
    }




}
