
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class rightbluefull {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d blueStart = new Pose2d(12,58,Math.toRadians(270));
        Pose2d redStart = new Pose2d(12,-58,Math.toRadians(90));

        Pose2d thirdBlueStripe = new Pose2d(24,38,Math.toRadians(270));
        Pose2d secondBlueStripe = new Pose2d(12, 32, Math.toRadians(270));
        Pose2d firstBlueStripe = new Pose2d(0, 38, Math.toRadians(270));

        Pose2d thirdRedStripe = new Pose2d(24,-38,Math.toRadians(90));
        Pose2d secondRedStripe = new Pose2d(12, -32, Math.toRadians(90));
        Pose2d firstRedStripe = new Pose2d(0, -38, Math.toRadians(90));

        Pose2d thirdBlueScoreArea = new Pose2d(50,41,Math.toRadians(0));
        Pose2d secondBlueScoreArea = new Pose2d(50,35, Math.toRadians(0));
        Pose2d firstBlueScoreArea = new Pose2d(50, 29, Math.toRadians(0));

        Pose2d thirdRedScoreArea = new Pose2d(50,-29, Math.toRadians(0));
        Pose2d secondRedScoreArea = new Pose2d(50,-35,Math.toRadians(0));
        Pose2d firstRedScoreArea = new Pose2d(50,-41,Math.toRadians(0));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStart)
                                .lineToLinearHeading(firstBlueStripe)
                                .addTemporalMarker(0,()->{

                                })
                                .waitSeconds(2)
                                .lineToLinearHeading(firstBlueScoreArea)
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{

                                })
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(14.6,5.5,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-54,5.5,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-54,10.5,Math.toRadians(0)))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}



