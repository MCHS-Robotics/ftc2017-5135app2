package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 1/30/18.
 */

@Autonomous(name="Auto_Blue", group="Linear Opmode")
public class BlueSideAuto extends GeneralAuto {

    @Override
    protected void knockOffJewel(boolean red) {
        if (red) {
            move.pivotRight(90);
        } else {
            move.pivotLeft(270);
        }
    }

    @Override
    protected void scoreGlyphAndPark() {
       // move.forward(24);
        //move.pivotLeft(90);
        //move.backward(12);
        //move.pivotLeft(90);
        //move.forward(10);
    }
}
