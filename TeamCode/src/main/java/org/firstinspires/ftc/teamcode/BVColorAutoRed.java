package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BVColorProcessor.merge;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.redContourArea;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.redProcessor;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.contoursRed;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BVColorAutoRed extends LinearOpMode {

    final Scalar BLUE = new Scalar(0, 255, 0);

    OpenCvWebcam webcam;

    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(redProcessor);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (opModeInInit()) {
            for (int i = 0; i < contoursRed.size(); i++) {
                Imgproc.drawContours(merge, contoursRed, i, BLUE, 2, Imgproc.LINE_8);
            }
        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Detecting RED Contours");
            telemetry.addData("Contours Detected", contoursRed.size());

            for (int i = 0; i < contoursRed.size(); i++) {
                telemetry.addData("Contour Points: ", contoursRed);
            }
            if (redContourArea > 1000 && gamepad1.x) {
                telemetry.addData("Element Detected! Area of Element:", redContourArea);
            }
        }
    }
}
