package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class BVWebCamTest extends LinearOpMode {

    // Method for aprilTag init: initAprilTagProcessor()
    // Method for tfod init: initTfodProcessor

    //Global VisionPortal for both processors
    VisionPortal webCam;

    //aprilTagProcessor for aprilTag init
    AprilTagProcessor tagProcessor;

    //tfodProcessor for tfod init
    TfodProcessor tfodProcessor;


    @Override
    public void runOpMode() throws InterruptedException {

        //Put apriltag + tfod inits here
        initAprilTagProcessor();
        initTfodProcessor();

        waitForStart();

        while (opModeIsActive()) {

            //Combine telemetry here

        }

    }

    //Put methods for apriltag + tfod inits here
    private void initAprilTagProcessor() {

        //April tag processor init
        tagProcessor = new AprilTagProcessor.Builder()

                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                .build();

        // Vision Portal / Camera init
        VisionPortal.Builder webCamBuilder = new VisionPortal.Builder();
        webCamBuilder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        webCamBuilder.setCameraResolution(new Size(800, 600));

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        webCamBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        webCamBuilder.setAutoStopLiveView(true);

        // Set and enable the processor.
        webCamBuilder.addProcessor(tagProcessor);

        // Build the Vision Portal, using the above settings.
        webCam = webCamBuilder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //webCam.setProcessorEnabled(aprilTag, true);

    }

    private void initTfodProcessor() {

        // Create the TensorFlow processor by using a builder.
        tfodProcessor = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(40)
                .setModelAspectRatio(16.0 / 12.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the webcam.
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640,480));

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfodProcessor);

        // Build the Vision Portal, using the above settings.
        webCam = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

}
