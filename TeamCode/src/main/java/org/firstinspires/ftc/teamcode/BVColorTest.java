package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Camera;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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
public class BVColorTest extends LinearOpMode{

    //Global vars

    //Color constants

    //RGB

    final Scalar BLUE = new Scalar(0, 0, 255);
    final Scalar RED = new Scalar(255, 0, 0);
    final Scalar PURPLE = new Scalar(255, 0, 255);
    final Scalar GREEN = new Scalar(0, 255, 0);

    //HSV

    final Scalar LOW_BLUE = new Scalar(100, 100, 100);
    final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

    List<MatOfPoint> contours = new ArrayList<>();

    //Stores the converted RGB to HSV Mat
    Mat hsvMat = new Mat();
    //Stores a 'bitmap' of the values in range of color
    Mat inRangeMat = new Mat();
    //Designs how the morph var is stored
    Mat kernel = Mat.ones(7, 7, CvType.CV_8U);
    //Stores the morphed Mat which has most sound removed
    Mat morph = new Mat();
    //Stores the information of a contours' image topology, unused
    Mat hierarchy = new Mat();
    //Output; stores the output
    Mat output = new Mat();

    //--------------------------------------------------------

    //Webcam initialization
    OpenCvWebcam webcam;

    //Pipeline initialization
    OpenCvPipeline Processor = new OpenCvPipeline() {

        @Override
        public Mat processFrame(Mat input) {

            //Variables for processFrame()

            //-----------------------------------------------------

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

            BVColorTest.this.contours = contours;

            //Returns input to webcam
            return input;
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.setPipeline(Processor);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Log.d("OPENCV", "WEBCAM STARTED STREAMING");
                webcam.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        while (opModeInInit()) {for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(output, contours, i, RED, 1, 1);
            }
        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Contours Detected", contours.size());

            List<MatOfPoint> contours = this.contours;
            for (int i = 0; i < contours.size(); i++) {
                telemetry.addData("Contour Points: ", contours);
            }

            telemetry.update();
        }
    }
}