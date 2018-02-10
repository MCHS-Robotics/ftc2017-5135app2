package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 1/30/18.
 */

@Autonomous(name="Auto_Red", group="Linear Opmode")
public class RedSideAuto extends GeneralAuto {

    @Override
    protected void knockOffJewel(boolean red) {
        if (red) {
            move.pivotLeft(90);
            jewel.setPosition(0);
            sleep(250);
        } else {
            move.pivotRight(30);
            jewel.setPosition(0);
            sleep(250);
            move.pivotLeft(120);
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
