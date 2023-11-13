package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TempBVClass extends LinearOpMode {
    public void runOpMode(){
        BVColorTest color = new BVColorTest(this);

        BVColorTest.SkystoneDeterminationPipeline.TSEposition barcode = color.getBarcode();

        waitForStart();

        while (opModeIsActive()) {

            if (barcode == BVColorTest.SkystoneDeterminationPipeline.TSEposition.TOP) {
                telemetry.addLine("No color");
            } else {
                telemetry.addLine("Color sensed");
            }
            telemetry.update();
        }
    }
}