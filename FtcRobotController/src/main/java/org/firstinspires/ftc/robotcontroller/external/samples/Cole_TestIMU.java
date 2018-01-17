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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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

@Autonomous(name="Test IMU", group="Linear Opmode")
public class Cole_TestIMU extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    DcMotor leftDrive;
    DcMotor rightDrive;

    Position pos;
    Velocity vel;
    Acceleration grav;
    Acceleration accel;
    Acceleration anet;

    Orientation angOrien;
    AngularVelocity L;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (pos.z > -1) {
                update();
                leftDrive.setPower(-0.3);
                rightDrive.setPower(-0.3);
            }
        }

        imu.stopAccelerationIntegration();
    }

    private void update() {
        // update IMU values
        pos = imu.getPosition();
        vel = imu.getVelocity();
        grav = imu.getGravity();
        accel = imu.getLinearAcceleration();
        anet = imu.getOverallAcceleration();

        angOrien = imu.getAngularOrientation();
        L = imu.getAngularVelocity();

        telemetry.addLine("Position: ");
        telemetry.addData("x: ", pos.x);
        telemetry.addData("y: ", pos.y);
        telemetry.addData("z: ", pos.z);
        telemetry.addLine("Velocity: ");
        telemetry.addData("x: ", vel.xVeloc);
        telemetry.addData("y: ", vel.yVeloc);
        telemetry.addData("z: ", vel.zVeloc);
        telemetry.addLine("Gravity: ");
        telemetry.addData("x: ", grav.xAccel);
        telemetry.addData("y: ", grav.xAccel);
        telemetry.addData("z: ", grav.xAccel);
        telemetry.addLine("From Robot: ");
        telemetry.addData("x: ", accel.xAccel);
        telemetry.addData("y: ", accel.xAccel);
        telemetry.addData("z: ", accel.xAccel);
        telemetry.addLine("Net: ");
        telemetry.addData("x: ", anet.xAccel);
        telemetry.addData("y: ", anet.xAccel);
        telemetry.addData("z: ", anet.xAccel);
        telemetry.addLine("Angular Velocity: ");
        telemetry.addData("x: ", L.xRotationRate);
        telemetry.addData("y: ", L.yRotationRate);
        telemetry.addData("z: ", L.zRotationRate);
        telemetry.addLine("Orientation: ");
        telemetry.addData("Ang1", angOrien.firstAngle);
        telemetry.addData("Ang2", angOrien.secondAngle);
        telemetry.addData("Ang3", angOrien.thirdAngle);
        telemetry.addData("Order", angOrien.axesReference.name());
        telemetry.update();

    }

}
