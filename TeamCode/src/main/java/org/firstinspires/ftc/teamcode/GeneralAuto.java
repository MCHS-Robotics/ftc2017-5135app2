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

public abstract class GeneralAuto extends LinearOpMode {

    // Declare OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor left = null;
    protected DcMotor right = null;
    protected DcMotor lift = null;
    protected CRServo pincher = null;
    protected Servo jewel = null;
    protected ColorSensor colorSensor = null;
    protected MovementStrategy move;

    public static final float power = 0.15f;

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


        move = new NormalDriveTime(right, left, telemetry, power);


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
        //raise();
        jewel.setPosition(0.55);
        sleep(250);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Position", jewel.getPosition());
        left.setPower(-power);
        right.setPower(-power);
        colorSensor.enableLed(true);
        // until a color is detected or 2 seconds have elapsed
        while (runtime.milliseconds() < 2000 && Math.abs(colorSensor.red() - colorSensor.blue()) < 25 && !isStopRequested()) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
        if (!isStopRequested()) {
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // knocks off jewel if detected
            if (Math.abs(colorSensor.red() - colorSensor.blue()) >= 20)
                knockOffJewel(colorSensor.red() > colorSensor.blue());
        }
        jewel.setPosition(0);
        sleep(250);

        scoreGlyphAndPark();
    }

    protected abstract void knockOffJewel(boolean red);
    protected  abstract  void scoreGlyphAndPark();

    protected void setPinch(boolean open)
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

    protected void lower()
    {
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

    protected void raise()
    {
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

