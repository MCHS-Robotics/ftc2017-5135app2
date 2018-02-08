package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

    public class NormalDriveTime implements MovementStrategy {

        final private int encoder = 1120;
        final private float turnRadius =  16.9f;

        private DcMotor left, right;
        private Telemetry telemetry;
        float power;

        public NormalDriveTime(DcMotor left, DcMotor right, Telemetry telemetry, float power) {
            this.left = left;
            this.right = right;
            this.telemetry = telemetry;
            this.power = power;
        }

        public void forward(float in) {
            left.setPower(power);
            right.setPower(power);
            try {
                Thread.sleep(1500);
            } catch (Exception e) {

            }
            left.setPower(0);
            right.setPower(0);
        }
        public void backward(float in)
        {

            left.setPower(-power);
            right.setPower(-power);
            try {
                Thread.sleep(1500);
            } catch (Exception e) {

            }
            left.setPower(0);
            right.setPower(0);
        }
        public void pivotLeft(float degrees)

        {

            left.setPower(power);
            right.setPower(-power);
            try {
                Thread.sleep(1500);
            } catch (Exception e) {

            }
            left.setPower(0);
            right.setPower(0);
         /*
        double arc = Math.PI * turnRadius * degrees / 360f;
        int pos = (int)((encoder * arc)/(4 * Math.PI));

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(pos);
        right.setTargetPosition(-pos);
        left.setPower(power);
        right.setPower(power);
//        if (pos < 0)
//            right.setPower(power);
//        else
//            right.setPower(-power);
        while(left.isBusy()/* && right.isBusy()*///)
         /*
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
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
        }
        public void pivotRight(float degrees)
        {

            left.setPower(-power);
            right.setPower(power);
            try {
                Thread.sleep(1500);
            } catch (Exception e) {

            }
            right.setPower(0);
            left.setPower(0);

            // pivotLeft(-degrees);
        }
        public void setPower(float a){power = a;}
    }
