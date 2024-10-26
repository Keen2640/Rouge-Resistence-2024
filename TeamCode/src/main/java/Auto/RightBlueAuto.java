package Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import Mechanisms.Camera;
import Mechanisms.Outake;
import Mechanisms.Outake.OutakeStates;

@Autonomous(name = "RightBlueAuto")
public class RightBlueAuto extends LinearOpMode {

    enum States {
        START,
        IDLE
    }

    States currentState = States.START;
    SampleMecanumDrive drive;

    Camera.BluePropDetector cam;
    Outake outake;



    public void initialize() {

        cam = new Camera.BluePropDetector(hardwareMap,telemetry);
        //uncomment this run pipline and get rid of the rest of the camera code if camera works fine and fov is good enough to capture both
        cam.runPipeline();
        //outake = new Outake(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void runOpMode() {
        int propPos = 0;
        initialize();
        Pose2d blueStart = new Pose2d(12,61,Math.toRadians(90));
        Pose2d blueCam = new Pose2d(20,61,Math.toRadians(90));

        Pose2d thirdBlueStripe = new Pose2d(24, 38, Math.toRadians(90));
        Pose2d secondBlueStripe = new Pose2d(12, 32, Math.toRadians(90));
        Pose2d firstBlueStripe = new Pose2d(0, 38, Math.toRadians(90));

        Pose2d thirdBlueScoreArea = new Pose2d(50, 41, Math.toRadians(180));
        Pose2d secondBlueScoreArea = new Pose2d(50, 35, Math.toRadians(180));
        Pose2d firstBlueScoreArea = new Pose2d(50, 29, Math.toRadians(180));

        drive.setPoseEstimate(blueStart);


        TrajectorySequence firstBlueTraj = drive.trajectorySequenceBuilder(blueCam)
                .lineToLinearHeading(firstBlueStripe)
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    /*
                       outake.setTargetPosition(500);
                       outake.currentOutakeState = OutakeStates.EXTENSION; */
                       outake.setTargetPosition(0);
                       outake.currentOutakeState = OutakeStates.EXTENSION;

                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    /*
                       outake.setTargetPosition(500);
                       outake.currentOutakeState = OutakeStates.EXTENSION; */
                    outake.setTargetPosition(0);
                    outake.currentOutakeState = OutakeStates.EXTENSION;
                    outake.openClawU();

                })
                .lineToLinearHeading(firstBlueScoreArea)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    /*
                       outake.setTargetPosition(500);
                       outake.currentOutakeState = OutakeStates.EXTENSION; */
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                  /*  outake.openClawU();
                    outake.setTargetPosition(0);
                    outake.currentOutakeState = OutakeStates.RESET; */
                })
                .waitSeconds(2)
                //parking
                .lineToLinearHeading(new Pose2d(50, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 10, Math.toRadians(0)))
                .build();

        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(blueCam)
                .lineToLinearHeading(secondBlueStripe)
                .waitSeconds(1)
                .lineToLinearHeading(secondBlueScoreArea)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                            /*
                       outake.setTargetPosition(500);
                       outake.currentOutakeState = OutakeStates.EXTENSION; */

                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                    outake.openClawB();
                    outake.setTargetPosition(0);
                    outake.currentOutakeState = OutakeStates.RESET; */
                })
                .waitSeconds(2)
                //parking
                .lineToLinearHeading(new Pose2d(50, 14, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 14, Math.toRadians(0)))
                .build();

        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(blueCam)
                .lineToLinearHeading(thirdBlueStripe)
                .addTemporalMarker(0, () -> {
                    //outake.openClawB();
                })
                .waitSeconds(2)
                .lineToLinearHeading(thirdBlueScoreArea)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    // outake.setTargetPosition(500);
                    // outake.currentOutakeState = OutakeStates.Extension;

                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    /*
                     /*
                    outake.openClawB();
                    outake.setTargetPosition(0);
                    outake.currentOutakeState = OutakeStates.RESET; */
                })
                .waitSeconds(2)
                //parking
                .lineToLinearHeading(new Pose2d(50, 14, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 14, Math.toRadians(0)))
                .build();

        waitForStart();
        //running pipeline after start button is pressed, its a bet, but have to experiement and see if calling runpipline once
        // before the command waitForStart(); causes the camera to be running the entire auto, or stops after init
        //cam.runPipeline();

        //drive.followTrajectorySequenceAsync(camDetection);
        currentState = States.START;

        while (opModeInInit()) {
            drive.setPoseEstimate(blueStart);
            propPos = cam.returnPropPos();
        }

        while (opModeIsActive()) {
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {
                        //after camDetection trajectory is over, we set propPos to the value stored in the camera object
                        //propPos = cam.returnPropPos();
                        //drive.followTrajectorySequenceAsync(firstBlueTraj);
                        switch (propPos) {
                            //after bot chooses trajectory based off of proppos,it runs that trajectory async and switches the state to idle
                            //so nothing happens after following through with the proper trajectory
                            case 1:
                                drive.followTrajectorySequenceAsync(firstBlueTraj);
                                currentState = States.IDLE;
                                break;
                            case 2:
                                drive.followTrajectorySequenceAsync(secondBlueTraj);
                                currentState = States.IDLE;
                                break;
                            case 3:
                                drive.followTrajectorySequenceAsync(thirdBlueTraj);
                                currentState = States.IDLE;
                                break;
                        }

                    }
                    break;
                case IDLE:
                    break;
            }
            outake.executeAuto();
            drive.update();
        }

    }
}
