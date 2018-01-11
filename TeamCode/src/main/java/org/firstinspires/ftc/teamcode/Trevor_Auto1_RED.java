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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


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

@Autonomous(name="Trevor_Auto1_RED", group="Linear Opmode")

public class Trevor_Auto1_RED extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor lift = null;
    private CRServo pincher = null;
    private Servo jewel = null;
    private ColorSensor colorSensor = null;
    final private int encoder = 1120;
    final private float turnRadius =  16.9f;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "fL");
        right = hardwareMap.get(DcMotor.class, "fR");
        pincher = hardwareMap.crservo.get("pincher");
        lift = hardwareMap.dcMotor.get("lift");
        jewel = hardwareMap.servo.get("jewel");
        colorSensor = hardwareMap.colorSensor.get("color");
        right.setDirection(DcMotor.Direction.REVERSE);
        pincher.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AWrTLxn/////AAAAGR6wTueSh0e5nXohk/1mFhhxlpeNrb42FL6M45v6X/OY10YsoKBuWg631uY8mZL9E3eoZvRadFq+K8oQFzwhYrLl+KfifFyOf/FO357kuymZaqGdpjRFgURHPe6LnL+KJb8gpUD2UTJ/nvdHFsbUJwQg+5ldrY9oQRVQ4y3RFazGDV/c5ZNHJC2jGj0Nkd9sx+VQQ+xKhyTASCWwKIDO/XYytI/7b8t9Pg+Bjb+AawM58VHpzD7ZtiWVpWQBA5QTGhBRq1u2rncx4E8plAs7kY7odfQuYUncRPM+PiEJFHi2F1lHHGXoarkzHpVeFLwO9AdhkCjw7AjH1ClBYCcKhsG2DicEXlRV2BtEyfh6ZOhE";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        jewel.setPosition(0.55);
        telemetry.addData("Position", jewel.getPosition());

        colorSensor.enableLed(true);
        // until a color is detected
        while (Math.abs(colorSensor.red() - colorSensor.blue()) < 25) {
            backward(1.25f);
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
        // Only works on one side (blue?)
        if (colorSensor.red() > colorSensor.blue()) {
            // red
            pivotRight(90);
            telemetry.addData("Red", "True");
            telemetry.update();
        } else {
            pivotLeft(90);
            // blue
            telemetry.addData("Blue", "True");
            telemetry.update();
        }
        colorSensor.enableLed(false);
        jewel.setPosition(0);
        try {
            Thread.sleep(1000);
        } catch(Exception e) {}
    }

    private void forward(float in)
    {
        int pos = (int)((encoder * in)/(4 * Math.PI));

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(pos);
        right.setTargetPosition(pos);
        left.setPower(.5);
        right.setPower(.5);
        while(left.isBusy() && right.isBusy())
        {
            telemetry.addData("Motor Encoder", "Left Pos: " + left.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Motor Encoder", "Right Pos: " + right.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Power","Left Pow: " + left.getPower());
            telemetry.addLine();
            telemetry.addData("Power","Right Pow: " + right.getPower());
            telemetry.addLine();
            telemetry.addData("Target","Left Tar: " + left.getTargetPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
        telemetry.update();
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void backward(float in)
    {
        forward(-in);
    }
    private void pivotLeft(float degrees)
    {
        double arc = Math.PI * turnRadius * degrees / 360f;
        int pos = (int)((encoder * arc)/(4 * Math.PI));

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(pos);
        right.setTargetPosition(-pos);
        left.setPower(.5);
        right.setPower(.5);
        while(left.isBusy() && right.isBusy())
        {
            telemetry.addData("Motor Encoder", "Left Pos: " + left.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Motor Encoder", "Right Pos: " + right.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Power","Left Pow: " + left.getPower());
            telemetry.addLine();
            telemetry.addData("Power","Right Pow: " + right.getPower());
            telemetry.addLine();
            telemetry.addData("Target","Left Tar: " + left.getTargetPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
        telemetry.update();
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void pivotRight(float degrees)
    {
        pivotLeft(-degrees);
    }
    private void setPinch(boolean open)
    {
        if(open)
        {
            pincher.setPower(0.5);
            try {
                Thread.sleep(1000);
            } catch (Exception e) {}
            pincher.setPower(0);
        }
        else
        {
            pincher.setPower(-0.5);
            try {
                Thread.sleep(1000);
            } catch (Exception e) {}
            pincher.setPower(0);
        }
    }

    private void lower() {
        lift.setPower(-0.1);
        try {
            for (int i = 0; i < 10; i++) {
                telemetry.addData("Pos", lift.getCurrentPosition());
                telemetry.update();
                Thread.sleep(1000);
            }
        } catch (Exception e) {}
        lift.setPower(0);
        telemetry.addData("Pos", lift.getCurrentPosition());
        telemetry.update();
    }

    private void raise() {
        lift.setPower(0.3);
        try {
            for (int i = 0; i < 10; i++) {
                telemetry.addData("Pos", lift.getCurrentPosition());
                telemetry.update();
                Thread.sleep(1000);
            }
        } catch (Exception e) {}
        lift.setPower(0);
        telemetry.addData("Pos", lift.getCurrentPosition());
        telemetry.update();
    }
}

