package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class BVColorAutoBlue extends LinearOpMode {

    final Scalar RED = new Scalar(255, 0, 0);
    final Scalar GREEN = new Scalar(0, 255, 0);

    //HSV Blue
    final Scalar LOW_BLUE = new Scalar(100, 100, 100);
    final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

    List<MatOfPoint> contoursBlue = new ArrayList<>();

    //Stores the converted RGB to HSV Mat
    Mat hsvMat = new Mat();
    //Stores a 'bitmap' of the values in range of color
    Mat inRangeMat = new Mat();
    //Designs how the morph var is stored
    Mat kernel = Mat.ones(7, 7, CvType.CV_8UC1);
    //Stores the morphed Mat which has most sound removed
    Mat morph = new Mat();
    //Stores the information of a contours' image topology, unused
    Mat hierarchy = new Mat();
    //contourMinimum range for the teleop controls
    int contourMinimum = 10000;

    //--------------------------------------------------------

    public void runOpMode() throws InterruptedException {

    OpenCvWebcam webcam;

    OpenCvPipeline blueProcessor = new OpenCvPipeline() {

        @Override
        public Mat processFrame(Mat input) {

            //Converts all color from RGB to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            //Creates a bitmap based on if the color is within the two scalar values
            Core.inRange(hsvMat, LOW_BLUE, HIGH_BLUE, inRangeMat);

            //Removes excess sound to create contours easily
            Imgproc.morphologyEx(inRangeMat, morph, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_OPEN, kernel);

            //Creates a list (array) of contours based on the now morphed image
            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(morph, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            BVColorAutoBlue.this.contoursBlue = contours;

            for (int i = 0; i < contours.size(); i++) {
                //Comment out the if then statement below to draw all contours
                //Note that all contours are detected in telemetry regardless
                if (Math.abs(Imgproc.contourArea(contoursBlue.get(i))) > contourMinimum) {

                    Imgproc.drawContours(input, contoursBlue, i, GREEN, 5, 2);

                    Rect rect = Imgproc.boundingRect(contours.get(i));
                    Imgproc.rectangle(input, rect, GREEN);
                }
                //Extra if then statement in order to view contours that are out of range
                if (Math.abs(Imgproc.contourArea(contoursBlue.get(i))) < contourMinimum) {
                    Imgproc.drawContours(input, contoursBlue, i, RED, 5, 2);
                }
            }

            //Returns input to webcam
            return input;
        }
    };

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

        @Override
        public void onOpened() {
            telemetry.addLine("INTIALIZATION SUCCESSFUL");
            telemetry.update();

            webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {
            telemetry.addData("ERROR UPON INITIALIZATION:", errorCode);
            telemetry.update();
        }
    });

    waitForStart();

    while (opModeIsActive()) {

        List<MatOfPoint> contoursBlue = BVColorAutoBlue.this.contoursBlue;

        webcam.setPipeline(blueProcessor);

        telemetry.addLine("Detecting BLUE Contours");
        telemetry.addData("Contours Detected", contoursBlue.size());
        telemetry.addData("Contour Minimum Vision", contourMinimum);

        if (gamepad1.right_stick_y > 0.3) {
            contourMinimum -= 1;
        } if (gamepad1.right_stick_y < -0.3) {
            contourMinimum += 1;
        }

        for (int i = 0; i < contoursBlue.size(); i++) {
            //If then statement to clear out unnecessary contours
            if (Imgproc.contourArea(contoursBlue.get(i)) > contourMinimum) {
                telemetry.addData("Element Detected! Area of Element:", Imgproc.contourArea(contoursBlue.get(i)));
            } else {
                telemetry.addData("Non-Element Contour Area:", Imgproc.contourArea(contoursBlue.get(i)));
            }
        }

        telemetry.update();

        }
    }
}
