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
            jewel.setPosition(0);
            sleep(250);
        } else {
            move.pivotLeft(30);
            jewel.setPosition(0);
            sleep(250);
            move.pivotRight(120);
        }
    }

    @Override
    protected void scoreGlyphAndPark() {
        move.forward(36);
        //move.pivotLeft(90);
        //move.backward(12);
        //move.pivotLeft(90);
        //move.forward(10);
    }
}
