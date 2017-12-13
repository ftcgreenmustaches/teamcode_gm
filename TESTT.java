package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by GM on 11/1/2017.
 * Program name : Blue Far
 * Purpose : This Autonomous program is from Blue Far
 */

@Autonomous(name="TESTT", group="Linear Opmode")
//@Disabled
public class TESTT extends LinearOpMode {

    private BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;
    private Servo servotest;
    private ColorSensor colorSensor=null;
    private DcMotor backmotorleft=null;
    private DcMotor backmotorright=null;
    private VuforiaTrackable relicTemplate = null;
    VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "motorleft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorright");
        armDrive1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armDrive2 = hardwareMap.get(DcMotor.class, "armmotor2");
        servotest = hardwareMap.get(Servo.class, "servotest");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        backmotorleft = hardwareMap.get(DcMotor.class, "backmotorleft");
        backmotorright = hardwareMap.get(DcMotor.class, "backmotorright");

        //
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        leftDrive.setDirection(Direction.FORWARD);
        rightDrive.setDirection(Direction.REVERSE);
        armDrive2.setDirection(Direction.FORWARD);
        armDrive1.setDirection(Direction.FORWARD);
        backmotorleft.setDirection(Direction.REVERSE);
        backmotorright.setDirection(Direction.FORWARD);

        parameters.vuforiaLicenseKey = "AZFp8MH/////AAAAGYv27PxrpU4kmiie6aqumwcN1emumBfetPrzCm5/O7j84" +
                "KMIBLToY69YTGzl6OQoST3BZN0c/SyA/tuug4FBbukF8tdC38mjyoUF1F4JwRtPtBpYAHGmlSe+cW" +
                "rE85Fl05x/IwHam+BMZFaq1Ov4h8xVIE1+7J9VQA0609IArWC/q5E9bKbzLK0yV2dZij61Nj8SSfF2O" +
                "dIwCStqzOiyZPUj5Ez9wFza5sJlEMhZbJUhyzXNtKZbolEe4Ilw5OAvlZjKoboDGBpp025ifax04ECVFD" +
                "wXrgMGzGDaPQHmepIigPRrmLfSl9LB/vGekFAvOnCJ4uy0jeeaeQTSDjC/LaB0W2x/M606KJSXdseMicCS";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        waitForStart();
        runtime.reset();
        relicTrackables.activate();
        while (opModeIsActive()) {
            sleep(2000);
            int vuTries = 700;
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while (vuMark == RelicRecoveryVuMark.UNKNOWN && vuTries > 0) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                vuTries -= 1;
            }
            final double HEADING_EPSILON = 1.5;
            final double TURN_SPEED = 0.25;

            if (vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("CENTER", RelicRecoveryVuMark.CENTER);
                telemetry.update();
                setDriveSpeed(0.3,0.3);
                sleep(2300);
                while (Math.abs(getHeading() + 90) > HEADING_EPSILON){
                    setDriveSpeed(TURN_SPEED, -TURN_SPEED);
                }
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("RIGHT", RelicRecoveryVuMark.RIGHT);
                telemetry.update();
                setDriveSpeed(0.3,0.3);
                sleep(1200);
                while (Math.abs(getHeading() + 90) > HEADING_EPSILON){
                    setDriveSpeed(TURN_SPEED, -TURN_SPEED);
                }
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("LEFT", RelicRecoveryVuMark.LEFT);
                telemetry.update();
                setDriveSpeed(0.3,0.3);
                sleep(3500);
                while (Math.abs(getHeading() + 90) > HEADING_EPSILON){
                    setDriveSpeed(TURN_SPEED, -TURN_SPEED);
                }
            } else {
                telemetry.addData("Unknown", 0);
                telemetry.update();
                setDriveSpeed(0.3,0.3);
                sleep(3500);
                while (Math.abs(getHeading() + 90) > HEADING_EPSILON){
                    setDriveSpeed(TURN_SPEED, -TURN_SPEED);
                }
            }

        }
    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    void setDriveSpeed(double left, double right) {
        leftDrive.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        rightDrive.setPower(right);
//        rightDrive2.setPower(right * 0.9);
    }
    void setBackSpeed (double left, double right) {
        backmotorleft.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        backmotorright.setPower(right);
//        rightDrive2.setPower(right * 0.9);
    }
}
