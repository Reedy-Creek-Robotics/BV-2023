package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.blueContourArea;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.blueProcessor;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.contoursBlue;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.webcam;

import org.opencv.core.MatOfPoint;

import java.util.List;

public class BVColorAutoBlue extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {

            waitForStart();

            while (opModeIsActive()) {

                telemetry.addLine("Detecting BLUE Contours");

                webcam.setPipeline(blueProcessor);

                telemetry.addData("Contours Detected", contoursBlue.size());

                for (int i = 0; i < contoursBlue.size(); i++) {
                    telemetry.addData("Contour Points: ", contoursBlue);

                    if (blueContourArea > 1000) {
                        telemetry.addData("Element Detected! Area of Element:", blueContourArea);
                    }
                }
            }
        }
    }
