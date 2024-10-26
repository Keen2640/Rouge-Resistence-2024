package Mechanisms;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import java.util.ArrayList;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.openftc.apriltag.AprilTagDetectorJNI;


public class Camera {
    public static class BlueConeDetector extends OpenCvPipeline {
        private final HardwareMap hardwareMap;
        private final Telemetry telemetry;
        Mat mat = new Mat(); // matrix
        final Rect LEFT_ROI = new Rect(
                new Point(1, 1),
                new Point(319, 479)); //takes the top left point and bottom right point ** 0,0 origin starts at top right
        final Rect RIGHT_ROI = new Rect(
                new Point(320, 1),
                new Point(639, 479));
        final double PERCENT_COLOR_THRESHOLD = 0.3; // unlike the HSV which determines what is considered our desired color, this decides if we have enough in the frame to actually perform an action
        public enum DETECTION_STATE {
            BOTH,
            LEFT,
            RIGHT,
            NOTFOUND
        }
        private DETECTION_STATE detection_state;


        private OpenCvCamera camera;

        public BlueConeDetector(Telemetry t, HardwareMap hM) {
            telemetry = t;
            hardwareMap = hM;
        }
        public void runPipeline() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
            //could potentially look at createInternalCamera
            camera.setPipeline(this);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    throw new RuntimeException("Error opening camera! Error code " + errorCode);
                }
            });
        }


        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // convert input into HSV and store in mat
            Scalar lowHSV = new Scalar(110, 50, 70);
            Scalar highHSV = new Scalar(130, 255, 255); //currently set to detect blue

            Core.inRange(mat, lowHSV, highHSV, mat); // the stored mat is thresholded into black and white and stored, white is desired color
            Mat left = mat.submat(LEFT_ROI); //extracting ROI by taking the sub matrix (part we want)
            Mat right = mat.submat(RIGHT_ROI);

            double leftValue = Core.sumElems(left).val[0]/ LEFT_ROI.area()/255;
            double rightValue = Core.sumElems(right).val[0]/ RIGHT_ROI.area()/255; // first we sum up all the elements which are basically pixels with different values in the matrix
            // the values I'm pretty sure represent how much of a channel it is, currently it's just gray. Then we divide by area to get average color for entire thing, and then make it out of 255 (max value for grayscale) to get decimal percentage
            // the higher the average submatrix value the more white is in the image
            left.release();
            right.release(); //opencv's stuff doesn't get automatically recycled so we need to do this to keep memory okay
            // the guy from yt video put release and then called them again, no idea what release actually does in technicality

            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue*100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue*100) + "%"); // debugging help

            boolean coneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean coneRight = rightValue > PERCENT_COLOR_THRESHOLD;

            if (coneLeft && coneRight) {
                // two cones
                detection_state = DETECTION_STATE.BOTH;
                telemetry.addData("Cone Location", "BOTH");
            }
            else if (coneLeft) {
                // one cone left
                detection_state = DETECTION_STATE.LEFT;
                telemetry.addData("Cone Location", "LEFT");
            }

            else if (coneRight){
                // one cone right
                detection_state = DETECTION_STATE.RIGHT;
                telemetry.addData("Cone Location", "RIGHT");
            }

            else {
                detection_state = DETECTION_STATE.NOTFOUND;
                telemetry.addData("Cone Location", "NOT FOUND");
            }
            telemetry.update();

            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB); // trying to draw some colored boxes around ROI
            Scalar colorNo = new Scalar(255, 0 ,0);
            Scalar colorYes = new Scalar(0, 255, 0);

            if (detection_state == DETECTION_STATE.BOTH) {
                Imgproc.rectangle(mat, LEFT_ROI, colorYes); // parameters go: what image, what rectangle, what color
                Imgproc.rectangle(mat, RIGHT_ROI, colorYes);
            }
            else if (detection_state == DETECTION_STATE.LEFT) {
                Imgproc.rectangle(mat, LEFT_ROI, colorYes);
                Imgproc.rectangle(mat, RIGHT_ROI, colorNo);
            }
            else if (detection_state == DETECTION_STATE.RIGHT) {
                Imgproc.rectangle(mat, RIGHT_ROI, colorYes);
                Imgproc.rectangle(mat, LEFT_ROI, colorNo);
            }
            else {
                Imgproc.rectangle(mat, RIGHT_ROI, colorNo);
                Imgproc.rectangle(mat, LEFT_ROI, colorNo);
            } // draws colored bounding box

            return mat;
        }

        public DETECTION_STATE getDetectionState() {
            return detection_state;
        }
    }










    // we can keep all detection in one class
    public static class DistanceEstimator extends OpenCvPipeline{
        double dist = 0;
        double focal = focalLength(9.55, 4, 282);
        private OpenCvCamera camera;
        private HardwareMap hardwareMap;
        private Telemetry telemetry;
        public DistanceEstimator(HardwareMap hw, Telemetry tele){
            this.hardwareMap = hw;
            this.telemetry = tele;
        }
        public void runPipeline() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
            //could potentially look at createInternalCamera
            camera.setPipeline(this);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    throw new RuntimeException("Error opening camera! Error code " + errorCode);
                }
            });
        }


        @Override
        public Mat processFrame(Mat input){
            try{       //requires try catch as if we don't find contours we get an error
                Mat end = input; //the frames that will be streamed
                Mat mat = input; //the frames that are altered to find our info

                Scalar lowHSV = new Scalar(110, 50, 70);
                Scalar highHSV = new Scalar(130, 255, 255);

                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
                Core.inRange(mat, lowHSV, highHSV, mat);  // thresholding

                Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, new Mat());
                Imgproc.blur(mat, mat, new Size(10, 10));// removing false negatives and using gaussian blur

                //finding all the contours in the image
                ArrayList<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                int largestIndex = 0;
                int largest = contours.get(0).toArray().length;

                //finding the largest contour form the frame
                for (int i = 0; i < contours.size(); i++) {
                    int currentSize = contours.get(i).toArray().length;
                    if (currentSize > largest) {

                        largest = currentSize;
                        largestIndex = i;
                    }

                }


                //Draw rectangle on largest contours

                MatOfPoint2f areaPoints = new MatOfPoint2f(contours.get(largestIndex).toArray());

                Rect rect = Imgproc.boundingRect(areaPoints);

                Imgproc.rectangle(end, rect, new Scalar(255, 0, 0));
                dist = estimateDist(focal, 0.6, rect.width);
                telemetry.addData("Distance: ", dist);
                telemetry.addData("pixle width", rect.width);
                telemetry.update();
                return end;
            } catch (IndexOutOfBoundsException e) {
                telemetry.addData("No objects found", 0);
            }
            return input;
        }
        //algos
        public static double focalLength(double measured_dist, double real_width, double width_in_rf_img ){
            return (width_in_rf_img * measured_dist)/ real_width;
        }
        public static double estimateDist(double focalL , double real_width, double width_in_rf_img ){
            return (focalL*real_width)/ width_in_rf_img;
        }
    }






    public static class BluePropDetector extends OpenCvPipeline{
        private OpenCvCamera camera;
        private HardwareMap hardwareMap;
        private Telemetry telemetry;
        private int loc = 0;
        private static double minArea = 30000;//20000.0


        public BluePropDetector(HardwareMap hw, Telemetry tele){
            this.hardwareMap = hw;
            this.telemetry = tele;
        }
        public void runPipeline() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
            //could potentially look at createInternalCamera
            camera.setPipeline(this);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    //camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    throw new RuntimeException("Error opening camera! Error code " + errorCode);
                }
            });
        }


        @Override
        public Mat processFrame(Mat input){
            try{       //requires try catch as if we don't find contours we get an error
                Mat end = input; //the frames that will be streamed
                Mat mat = input; //the frames that are altered to find our info

                Scalar lowHSV = new Scalar(110, 150, 70); //50
                Scalar highHSV = new Scalar(130, 255, 255);

                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
                Core.inRange(mat, lowHSV, highHSV, mat);  // thresholding

                Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, new Mat());
                Imgproc.blur(mat, mat, new Size(10, 10));// removing false negatives and using gaussian blur

                //finding all the contours in the image
                ArrayList<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                //filtering contours based off of area



                int largestIndex = 0;
                int largest = contours.get(0).toArray().length;

                //finding the largest contour form the frame

                for (int i = 0; i < contours.size(); i++) {
                    int currentSize = contours.get(i).toArray().length;
                    if (currentSize > largest) {

                        largest = currentSize;
                        largestIndex = i;
                    }

                }
                MatOfPoint2f areaPoints = new MatOfPoint2f(contours.get(largestIndex).toArray());

                //Draw rectangle on filtered contour



                Rect rect = Imgproc.boundingRect(areaPoints);
              //  RotatedRect rectR  = Imgproc.minAreaRect(areaPoints);

                if(rect.x==0 || rect.area()<3000) loc = 1;
                else if(rect.x<640) loc = 2;
                else loc = 3;
                Imgproc.rectangle(end, rect, new Scalar(255, 0, 0));
                telemetry.addData("area: ",rect.area());

                telemetry.addData("rect x: ", rect.x);
                telemetry.addData("ScoreLocation",loc);
                telemetry.update();
                return end;
            } catch (IndexOutOfBoundsException e) {
                loc = 1;
                telemetry.addData("No objects found Loc:", 1);
                telemetry.update();
            }
            return input;
        }
        // extra methods/algos
        public int returnPropPos(){
            return loc;
        }
        public void closeCamera(){
            camera.closeCameraDevice();
        }
    }
    public static class RedPropDetector extends OpenCvPipeline{
        double angleError;
        double focalLength = 360/Math.tan(Math.toRadians(48));
        private OpenCvCamera camera;
        private HardwareMap hardwareMap;
        private Telemetry telemetry;
        private static double minArea = 30000;//20000.0
        private int loc = 0;


        public RedPropDetector(HardwareMap hw, Telemetry tele){
            this.hardwareMap = hw;
            this.telemetry = tele;
        }
        public void runPipeline() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
            //could potentially look at createInternalCamera
            camera.setPipeline(this);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    //camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    throw new RuntimeException("Error opening camera! Error code " + errorCode);
                }
            });
        }


        @Override
        public Mat processFrame(Mat input){
            try{       //requires try catch as if we don't find contours we get an error
                Mat end = input; //the frames that will be streamed
                Mat mat = input; //the frames that are altered to find our info

                Scalar lowHSV = new Scalar(0, 50, 110); //S: 125
                Scalar highHSV = new Scalar(5, 255, 255);

                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
                Core.inRange(mat, lowHSV, highHSV, mat);  // thresholding

                Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, new Mat());
                Imgproc.blur(mat, mat, new Size(10, 10));// removing false negatives and using gaussian blur

                //finding all the contours in the image
                ArrayList<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                //filtering contours based off of area



                int largestIndex = 0;
                int largest = contours.get(0).toArray().length;

                //finding the largest contour form the frame

                for (int i = 0; i < contours.size(); i++) {
                    int currentSize = contours.get(i).toArray().length;
                    if (currentSize > largest) {

                        largest = currentSize;
                        largestIndex = i;
                    }

                }
                MatOfPoint2f areaPoints = new MatOfPoint2f(contours.get(largestIndex).toArray());

                //Draw rectangle on filtered contour



                Rect rect = Imgproc.boundingRect(areaPoints);
                //  RotatedRect rectR  = Imgproc.minAreaRect(areaPoints);


                if(rect.x==0 || rect.area()<3000) loc = 1;
                else if(rect.x<640) loc = 3;
                else loc = 2;
                Imgproc.rectangle(end, rect, new Scalar(255, 0, 0));
                telemetry.addData("area: ",rect.area());

                telemetry.addData("rect x: ", rect.x);
                telemetry.addData("ScoreLocation",loc);
                telemetry.update();
                return end;
            } catch (IndexOutOfBoundsException e) {
                loc=1;
                telemetry.addData("No objects found Loc:", 1);
                telemetry.update();
            }
            return input;
        }
        // extra methods/algos
        public int returnPropPos(){
            return loc;
        }
        public void closeCamera(){
            camera.closeCameraDevice();
        }
    }
    public static class AprilTagDetectionPipeline extends OpenCvPipeline {
        private long nativeApriltagPtr;
        private Mat grey = new Mat();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();

        private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
        private final Object detectionsUpdateSync = new Object();

        private Mat cameraMatrix;

        private final Scalar blue = new Scalar(7,197,235,255);
        private final Scalar red = new Scalar(255,0,0,255);
        private final Scalar green = new Scalar(0,255,0,255);
        private final Scalar white = new Scalar(255,255,255,255);

        private final double fx;
        private final double fy;
        private final double cx;
        private final double cy;

        // Units are in meters!
        private double tagsize;
        private double tagsizeX;
        private double tagsizeY;

        private float decimation;
        private boolean needToSetDecimation;
        private final Object decimationSync = new Object();

        public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy)
        {
            this.tagsize = tagsize;
            this.tagsizeX = tagsize;
            this.tagsizeY = tagsize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            constructMatrix();

            nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        public void finalize()
        {
            if(nativeApriltagPtr != 0)
            {
                AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
                nativeApriltagPtr = 0;
            }
            else
            {
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
            }
        }

        @Override
        public Mat processFrame(Mat input)
        {
            // Convert to grayscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync)
            {
                if(needToSetDecimation)
                {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag detection
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

            synchronized (detectionsUpdateSync)
            {
                detectionsUpdate = detections;
            }

            // Draw the pose of the AprilTags
            for(AprilTagDetection detection : detections)
            {
                Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
            }

            return input;
        }

        public void setDecimation(float decimation)
        {
            synchronized (decimationSync)
            {
                this.decimation = decimation;
                needToSetDecimation = true;
            }
        }

        public ArrayList<AprilTagDetection> getLatestDetections()
        {
            return detections;
        }

        public ArrayList<AprilTagDetection> getDetectionsUpdate()
        {
            synchronized (detectionsUpdateSync)
            {
                ArrayList<AprilTagDetection> ret = detectionsUpdate;
                detectionsUpdate = null;
                return ret;
            }
        }

        void constructMatrix()
        {
            //     Construct the camera matrix.
            //
            //      --         --
            //     | fx   0   cx |
            //     | 0    fy  cy |
            //     | 0    0   1  |
            //      --         --
            //

            cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

            cameraMatrix.put(0,0, fx);
            cameraMatrix.put(0,1,0);
            cameraMatrix.put(0,2, cx);

            cameraMatrix.put(1,0,0);
            cameraMatrix.put(1,1,fy);
            cameraMatrix.put(1,2,cy);

            cameraMatrix.put(2, 0, 0);
            cameraMatrix.put(2,1,0);
            cameraMatrix.put(2,2,1);
        }

        void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(0,0,0),
                    new Point3(length,0,0),
                    new Point3(0,length,0),
                    new Point3(0,0,-length)
            );

            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Draw the marker
            Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

            Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
        }

        void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(-tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2,-tagHeight/2,-length),
                    new Point3(-tagWidth/2,-tagHeight/2,-length));

            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            for(int i = 0; i < 4; i++)
            {
                Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
            }

            // Top lines
            Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
            Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
            Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
            Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
        }

        private Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
        {
            // The actual 2d points of the tag detected in the image
            MatOfPoint2f points2d = new MatOfPoint2f(points);

            // The 3d points of the tag in an 'ideal projection'
            Point3[] arrayPoints3d = new Point3[4];
            arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
            arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
            MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

            Pose pose = new Pose();
            Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

            return pose;
        }

        /*
         * A simple container to hold both rotation and translation
         * vectors, which together form a 6DOF pose.
         */
        class Pose
        {
            Mat rvec;
            Mat tvec;

            public Pose()
            {
                rvec = new Mat();
                tvec = new Mat();
            }

            public Pose(Mat rvec, Mat tvec)
            {
                this.rvec = rvec;
                this.tvec = tvec;
            }
        }
    }
    public static class TagDetector{
        // Units are in pixels
        private static final double fx = 578.272;
        private static final double fy = 578.272;
        private static final double cx = 402.145;
        private static final double cy = 221.506;
        // UNITS ARE METERS
        private static final double tagsize = 0.166;
        private double tagNumber = 0;

        //no idea what this is
        private int numFramesWithoutDetection = 0;
        private static final float DECIMATION_HIGH = 3;
        private static final float DECIMATION_LOW = 2;
        private static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        private static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

        private static final double FEET_PER_METER = 3.28084;

        private static double x_displacement = 0;
        private HardwareMap hardwareMap;
        private Telemetry telemetry;
        private OpenCvCamera camera;

        private AprilTagDetectionPipeline aprilTagDetectionPipeline;
        public TagDetector(HardwareMap hardwareMap, Telemetry telemetry){
            this.hardwareMap = hardwareMap;
            this.telemetry = telemetry;
        }
        public void runPipeline() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            //could potentially look at createInternalCamera
            camera.setPipeline(aprilTagDetectionPipeline);



            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    //camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    throw new RuntimeException("Error opening camera! Error code " + errorCode);
                }
            });
        }
        public void closeCamera(){
            camera.closeCameraDevice();
        }
        public void run(){
            try {
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

                if (detections != null) {
                    telemetry.addData("FPS", camera.getFps());
                    telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                    telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                    if (detections.size() == 0) {
                        numFramesWithoutDetection++;

                        if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        }
                    } else {
                        numFramesWithoutDetection = 0;

                        if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                        }

                        for (AprilTagDetection detection : detections) {
                            if(detection!=null){
                                x_displacement = detection.pose.x * FEET_PER_METER;
                            }
                            else{
                                x_displacement = 0.0;
                            }
                            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));


                            //...
                            if (detection.id == 1 || detection.id == 2 || detection.id == 3) {
                                tagNumber = detection.id;
                            }
                            //...
                        }
                    }
                    telemetry.update();

                }




                //PID CONSTANT CORRECTION OF SLIDES

                //PID ENDS HERE
            }
            catch (Exception e) {
                telemetry.addData("Status", "CAMERA ERROR, EXCEPTION CAUGHT");
                telemetry.update();
            }
        }
        public double returnTagXDispalcement(){
            return x_displacement;
        }

    }

    }



