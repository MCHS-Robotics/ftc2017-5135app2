package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by student on 1/30/18.
 */

public class NormalDriveIMU implements MovementStrategy {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    DcMotor left;
    DcMotor right;

    Position pos;
    Velocity vel;
    Acceleration grav;
    Acceleration accel;
    Acceleration anet;

    Orientation angOrien;
    AngularVelocity L;

    Telemetry telemetry;
    float power;

    public NormalDriveIMU(DcMotor left, DcMotor right, Telemetry telemetry, float power,HardwareMap hardwareMap) {

        this.left = left;
        this.right = right;
        this.telemetry = telemetry;
        this.power = power;

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        update();
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

    public float velocity(float power) {
        return 29f / 3000; //measured
    }
    @Override
    public void forward(float inches) {
        left.setPower(-power);
        right.setPower(-power);
        try {
            Thread.sleep(Math.round(inches / velocity(power)));
        } catch (Exception e) {

        }
        left.setPower(0);
        right.setPower(0);
    }
    @Override
    public void backward(float inches)
    {

        left.setPower(power);
        right.setPower(power);
        try {
            Thread.sleep(Math.round(inches / velocity(power)));
        } catch (Exception e) {

        }
        left.setPower(0);
        right.setPower(0);
    }

    @Override
    public void pivotLeft(float amount) {
        amount %= 360;
        amount -= 5;
        float start = angOrien.firstAngle;
        float finalAng = start + amount;
        if (finalAng > 180) {
            if (angOrien.firstAngle < 0) {
                while (angOrien.firstAngle < 0)
                    update();
            }
            while (angOrien.firstAngle >= 0) {
                update();
            }
            finalAng = start + amount - 360;
        }
        right.setPower(-power);
        left.setPower(power);
        while (angOrien.firstAngle < finalAng) {
            update();
        }
        right.setPower(0);
        left.setPower(0);
    }

    @Override
    public void pivotRight(float amount) {
        amount %= 360;
        amount -= 5;
        float start = angOrien.firstAngle;
        float finalAng = start - amount;
        right.setPower(power);
        left.setPower(-power);
        if (finalAng < -180) {
            if (angOrien.firstAngle > 0) {
                while (angOrien.firstAngle > 0) {
                    update();
                }
            }
            while (angOrien.firstAngle <= 0) {
                update();
            }
            finalAng = 360 - amount + start;
        }
        while (angOrien.firstAngle > finalAng) {
            update();
        }
        right.setPower(0);
        left.setPower(0);
    }

    public void moveForwardOffBalance() {
        right.setPower(power);
        left.setPower(power);
        while (grav.yAccel < 0.05);
        while (grav.yAccel > 0.05);
        right.setPower(0);
        left.setPower(0);
    }
}