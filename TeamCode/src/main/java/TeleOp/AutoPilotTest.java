package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import Mechanisms.Camera;
import Mechanisms.DriveTrain;

//@TeleOp(name = "AutoPilotTest")
public class AutoPilotTest extends LinearOpMode {
    Camera.BlueConeDetector blueConePipeline;


    Camera.BluePropDetector PropDetector;
  /*  private Servo rightServo;
    private Servo leftServo;
    DriveTrain dt; */
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    DriveTrain driveTrain;
    private void initialize() {
        dashboard.setTelemetryTransmissionInterval(25);
        driveTrain = new DriveTrain(hardwareMap,telemetry,gamepad1);
        PropDetector = new Camera.BluePropDetector(hardwareMap,telemetry);
        PropDetector.runPipeline();
       // driveTrain.setPose();
       /* blueConePipeline = new Camera.BlueConeDetector(telemetry, hardwareMap);
        blueConePipeline.runPipeline();

        dt = new DriveTrain(hardwareMap, gamepad1);

        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo"); */
    }




    @Override
    public void runOpMode() {
        initialize();
        while (opModeInInit()) {
            int i = PropDetector.returnPropPos();
        }
        while (opModeIsActive()) {
            driveTrain.drive();
        }
    }
}
