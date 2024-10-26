package Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import Mechanisms.Camera;
import Mechanisms.Outake;

public class RightRedAuto extends LinearOpMode {

    enum States{
        START,
        IDLE
    }
    States currentState = States.START;
    SampleMecanumDrive drive;
    Camera.RedPropDetector cam;
    Outake outake;


    public void initialize(){
        cam = new Camera.RedPropDetector(hardwareMap,telemetry);
        cam.runPipeline();
        outake = new Outake(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
    }
    @Override
    public void runOpMode(){
        int propPos = 0;
        Pose2d redStart = new Pose2d(12,-58,Math.toRadians(90));

        Pose2d thirdRedStripe = new Pose2d(24,-38,Math.toRadians(90));
        Pose2d secondRedStripe = new Pose2d(12, -32, Math.toRadians(90));
        Pose2d firstRedStripe = new Pose2d(0, -38, Math.toRadians(90));

        Pose2d thirdRedScoreArea = new Pose2d(50,-29, Math.toRadians(0));
        Pose2d secondRedScoreArea = new Pose2d(50,-35,Math.toRadians(0));
        Pose2d firstRedScoreArea = new Pose2d(50,-41,Math.toRadians(0));

        TrajectorySequence firstRedTraj = drive.trajectorySequenceBuilder(redStart)
                .lineToLinearHeading(firstRedStripe)
                .addTemporalMarker(0,()->{
                    outake.openClawB();
                })
                .waitSeconds(2)
                .lineToLinearHeading(firstRedScoreArea)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    outake.openClawU();
                    outake.setTargetPosition(0);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(50,-14,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-14,Math.toRadians(0)))
                .build();

        TrajectorySequence secondRedTraj = drive.trajectorySequenceBuilder(redStart)
                .lineToLinearHeading(secondRedStripe)
                .addTemporalMarker(0,()->{
                    outake.openClawB();
                })
                .waitSeconds(2)
                .lineToLinearHeading(secondRedScoreArea)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    outake.openClawU();
                    outake.setTargetPosition(0);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(50,-14,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-14,Math.toRadians(0)))
                .build();

        TrajectorySequence thirdRedTraj = drive.trajectorySequenceBuilder(redStart)
                .lineToLinearHeading(thirdRedStripe)
                .addTemporalMarker(0,()->{
                    outake.openClawB();
                })
                .waitSeconds(2)
                .lineToLinearHeading(thirdRedScoreArea)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    outake.openClawU();
                    outake.setTargetPosition(0);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(50,-14,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-14,Math.toRadians(0)))
                .build();




        while(opModeInInit()) {
             propPos = cam.returnPropPos();
        }

        while(opModeIsActive()) {
            switch(currentState){
                case START:
                    if (!drive.isBusy()) {
                            switch (propPos) {
                                case 1:
                                    drive.followTrajectorySequenceAsync(firstRedTraj);
                                    currentState = States.IDLE;
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(secondRedTraj);
                                    currentState = States.IDLE;
                                    break;
                                case 3:
                                    drive.followTrajectorySequenceAsync(thirdRedTraj);
                                    currentState = States.IDLE;
                                    break;
                        }

                    }
                    break;

                case IDLE:
                    break;
            }
           outake.runSlidesPID();
        }
    }
}
