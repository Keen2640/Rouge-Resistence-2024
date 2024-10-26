package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import Filters.KalmanF;
import Filters.LowPassF;
import org.ejml.simple.SimpleMatrix;
import Mechanisms.Camera;





/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.37795; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 0; // X is the up and down direction
    public static double PARALLEL_Y = 0; // Y is the strafe direction

    public static double PERPENDICULAR_X = -4.5;
    public static double PERPENDICULAR_Y = 3.7;

    public static double X_MULTIPLIER = 0.50126635139, Y_MULTIPLIER = 0.5044529498;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    private Camera.TagDetector tagDetector;

    private enum CamStates{
        CAM_ON,
        CAM_OFF
    }

    private CamStates camStates = CamStates.CAM_OFF;





    //KalmanFilter Filter
    //updates ever 50 miliseconds
    double dt = 0.025;
    private ElapsedTime runtime = new ElapsedTime();
    double nextUpdateTime = runtime.milliseconds() + dt * 1000;
    double lastEstimateTime = runtime.seconds();

    SimpleMatrix F = new SimpleMatrix(new double[][]{
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    });

    SimpleMatrix G = new SimpleMatrix(new double[][]{
            {0.5 * dt * dt},
            {0.5 * dt * dt},
            {dt},
            {dt}
    });

    double sigma_a = 0.005;
    SimpleMatrix Q = G.mult(G.transpose()).scale(sigma_a * sigma_a);

    SimpleMatrix H = SimpleMatrix.identity(F.numRows());

    double positionVar = 0.005;
    double velocityVar = 0.001;

    SimpleMatrix R = new SimpleMatrix(new double[][]{
            {positionVar, 0, 0, 0},
            {0, positionVar, 0, 0},
            {0, 0, velocityVar, 0},
            {0, 0, 0, velocityVar}
    });


    double a_x = 20;
    double a_y = 20;

    SimpleMatrix u_u = new SimpleMatrix(new double[][]{
            {a_x},
            {a_y},
            {a_x},
            {a_y}
    });

    SimpleMatrix u_0 = new SimpleMatrix(new double[][]{
            {0},
            {0},
            {0},
            {0}
    });

    SimpleMatrix updatedEst = new SimpleMatrix(new double[][]{
            {0},
            {0},
            {0},
            {0}
    });

    KalmanF robotKalmanFilter = new KalmanF(F, G, R, Q, H);

    double lastX, lastY;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, Telemetry telemetry) {

        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        SimpleMatrix x_0 = new SimpleMatrix(new double[][]{
                {drive.getPoseEstimate().getX()},
                {drive.getPoseEstimate().getY()},
                {0},
                {0}
        });
        SimpleMatrix p_0 = new SimpleMatrix(new double[][]{
                {0.1, 0, 0, 0},
                {0, 0.1, 0, 0},
                {0, 0, 0.1, 0},
                {0, 0, 0, 0.1}
        });
        //robotKalmanFilter.setInitalPostion(x_0, p_0, getHeading());
        this.drive = drive;

        /*parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder")); */
        this.drive = drive;
        tagDetector = new Camera.TagDetector(hardwareMap,telemetry);
        robotKalmanFilter.setInitalPostion(x_0, p_0, getHeading());
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {

        switch(camStates){
            case CAM_ON:
                tagDetector.run();
                double y = encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                        x = encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) + tagDetector.returnTagXDispalcement();
                break;
            case CAM_OFF:
                break;
        }
        double y = encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                x = encoderTicksToInches(perpendicularEncoder.getCurrentPosition());
        double dt_actual = runtime.seconds() - lastEstimateTime;
        lastEstimateTime = runtime.seconds();

        SimpleMatrix z_k = new SimpleMatrix(new double[][]{
                {x},
                {y},
                {(x - lastX) / dt_actual},
                {(y - lastY) / dt_actual}
        });
        lastX = x;
        lastY = y;

        SimpleMatrix v = velocity(z_k);
        double speed = v.normF();
        SimpleMatrix u = estimateControlInput(speed, 50); //steady state speed could be max speed
        robotKalmanFilter.setB(getHeading());
        updatedEst = robotKalmanFilter.update(z_k, u);

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())* Y_MULTIPLIER
        );
        /*
        return Arrays.asList(
                updatedEst.get(0,0),
                updatedEst.get(1,0)
        ); */
    }

    private SimpleMatrix estimateControlInput(double speed, int steadyStateSpeed) {
        SimpleMatrix u;
        if (speed < steadyStateSpeed) {
            u = u_u;
        } else {
            u = u_0;
        }
        return u;
    }
    private SimpleMatrix velocity(SimpleMatrix updatedEst) {
        return updatedEst.extractMatrix(2, 4, 0, 1);
    }

    public SimpleMatrix getEstimate(){
        return updatedEst;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
    public void turnCamOn(){
        camStates = CamStates.CAM_ON;
        tagDetector.runPipeline();

    }
    public void turnCamOff(){
        camStates = CamStates.CAM_OFF;
        tagDetector.closeCamera();
    }
}