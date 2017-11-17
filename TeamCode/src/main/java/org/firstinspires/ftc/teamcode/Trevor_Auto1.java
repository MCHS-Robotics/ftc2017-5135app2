/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Trevor_Auto1", group="Linear Opmode")

public class Trevor_Auto1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor lift = null;
    private CRServo pincher = null;
    final private int encoder = 1120;
    final private float turnRadius = (float) 16.9;
    /*
    RelicRecoveryVuMark vuMark1 = RelicRecoveryVuMark.from(relicTemplate);
    RelicRecoveryVuMark vuMark2 = RelicRecoveryVuMark.from(relicTemplate);
    RelicRecoveryVuMark vuMark3 = RelicRecoveryVuMark.from(relicTemplate);
    */

    @Override
    public void runOpMode() {

        left  = hardwareMap.get(DcMotor.class, "fL");
        right = hardwareMap.get(DcMotor.class, "fR");
        pincher = hardwareMap.crservo.get("pincher");
        lift = hardwareMap.dcMotor.get("lift");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        forward(19);
        pivotLeft(90);
        forward(10);
        /*scaning
        if(1){
            backward(10);
            pivotRight(90);
            forward(10);
            pivotLeft(90);
            forward(17);
        }
        else if(2){
            backward(10);
            pivotRight(90);
            forward(18);
            pivotLeft(90);
            forward(17);
        }
        else if(3){
            backward(10);
            pivotRight(90);
            forward(26);
            pivotLeft(90);
            forward(17);
        }
        else{
            backward(10);
            pivotRight(90);
            forward(18);
            pivotLeft(90);
            forward(17);
        }
        */

    }
    private void forward(float in)
    {
        int pos = (int)((encoder * in)/(4 * Math.PI));
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(pos);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setPower(.5);
        right.setPower(.5);
        while(left.isBusy()){
            telemetry.addData("Motor Encoder", "Left: " + left.getCurrentPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
    private void backward(float in)
    {
        int pos = (int)((encoder * in)/(4 * Math.PI));
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(-pos);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setPower(-.5);
        right.setPower(-.5);
        while(left.isBusy()){
            telemetry.addData("Motor Encoder", "Left: " + left.getCurrentPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
    private void pivotLeft(float degrees)
    {
        float arc = turnRadius * degrees;
        int pos = (int)((encoder * arc)/(4 * Math.PI));
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(-pos);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setPower(-.5);
        right.setPower(.5);
        while(left.isBusy()){
            telemetry.addData("Motor Encoder", "Left: " + left.getCurrentPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
    private void pivotRight(float degrees)
    {
        float arc = turnRadius * degrees;
        int pos = (int)((encoder * arc)/(4 * Math.PI));
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(-pos);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setPower(.5);
        right.setPower(-.5);
        while(left.isBusy()){
            telemetry.addData("Motor Encoder", "Left: " + left.getCurrentPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}

