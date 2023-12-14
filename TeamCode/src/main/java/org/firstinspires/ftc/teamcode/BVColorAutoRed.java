package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BVColorProcessor.redContourArea;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.redProcessor;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.webcam;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.contoursRed;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.MatOfPoint;

import java.util.List;

@Autonomous
public class BVColorAutoRed extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Detecting RED Contours");

            webcam.setPipeline(redProcessor);

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
