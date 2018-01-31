package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 1/30/18.
 */

@Autonomous(name="Auto_Blue", group="Linear Opmode")
public class BlueSideAuto extends GeneralAuto {

    protected void knockOffJewel(boolean red) {
        if (red) {
            move.pivotRight(180);
        } else {
            move.pivotLeft(180);
        }
    }

}
