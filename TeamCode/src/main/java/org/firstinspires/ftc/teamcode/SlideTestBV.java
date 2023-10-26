package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class SlideTestBV extends LinearOpMode {

    public DcMotor LinearSlide;

    public void runOpMode() {

        LinearSlide = hardwareMap.get(DcMotor.class,"linearSlide");

        LinearSlide.setDirection(DcMotor.Direction.FORWARD);

        LinearSlide.setZeroPowerBehavior(BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a || gamepad2.a) {
                LinearSlide.setPower(0.75);
            }
            if (gamepad1.x || gamepad2.x) {
                LinearSlide.setPower(0);

            }
            if (gamepad1.b || gamepad2.b ) {
                LinearSlide.setPower(-0.75);
            }
            telemetry.addLine("Use button A and B on either controller to control the linear slide.");
        }

    }

}
