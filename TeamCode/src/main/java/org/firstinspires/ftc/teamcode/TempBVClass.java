package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TempBVClass extends LinearOpMode {
    public void runOpMode(){
        BVColorTest color = new BVColorTest(this);

        BVColorTest.SkystoneDeterminationPipeline.TSEposition barcode = color.getBarcode();

        if (barcode == BVColorTest.SkystoneDeterminationPipeline.TSEposition.MID) {
            //dothing2
        }

        if(barcode == BVColorTest.SkystoneDeterminationPipeline.TSEposition.TOP){
            //dothing
        }
    }

}