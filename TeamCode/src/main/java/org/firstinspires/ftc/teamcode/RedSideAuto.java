package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 1/30/18.
 */

@Autonomous(name="Auto_Red", group="Linear Opmode")
public class RedSideAuto extends GeneralAuto {

    protected void knockOffJewel(boolean red) {
        if (red) {
            move.pivotLeft(180);
        } else {
            move.pivotRight(180);
        }
    }

}
