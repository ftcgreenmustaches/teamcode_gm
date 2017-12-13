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

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "REDCLOSEtest", group = "Sensor")
//@Disabled
public class REDCLOSEtest extends LinearOpMode {
    public static final double JEWEL_SPEED = 0.35;
    public static final int JEWEL_TIME = 500;
    public static final double CR_DOWN = -0.75;
    ColorSensor colorsensor2;    // Hardware Device Object
    ColorSensor colorsensor;    // Hardware Device Object
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor backmotorleft = null;
    private DcMotor backmotorright = null;
    private Servo servotest = null;
    private Servo servoturn = null;
    private BNO055IMU imu;
    private DcMotor armDrive1;
    private DcMotor armDrive2;

    //private TouchSensor digitaltouch=null;
    @Override
    public void runOpMode() {
        // leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);


        // get a reference to our ColorSensor object.
        colorsensor2 = hardwareMap.get(ColorSensor.class, "colorsensor2");
        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        leftDrive = hardwareMap.get(DcMotor.class, "motorleft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorright");
        backmotorleft = hardwareMap.get(DcMotor.class, "backmotorleft");
        backmotorright = hardwareMap.get(DcMotor.class, "backmotorright");
        servotest = hardwareMap.get(Servo.class, "servotest");
        servoturn = hardwareMap.get(Servo.class, "servoturn");
        armDrive1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armDrive2 = hardwareMap.get(DcMotor.class, "armmotor2");
        //digitaltouch=hardwareMap.get(TouchSensor.class, "digitaltouch")

        armDrive2.setDirection(DcMotor.Direction.FORWARD);
        armDrive1.setDirection(DcMotor.Direction.FORWARD);
        backmotorleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backmotorright.setDirection(DcMotorSimple.Direction.FORWARD);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ad2LNdb/////AAAAGR0zE9J++E4mgHVzCDy04EE6RLn++aUJb//ZYP" +
                "YlO/v9Zj6I8/oj2ye9joVQx5/vfBCfYy6VcF7zXivXb1xNf4P33bGQ2umP55eFRVMb/nVQsOyRUox9Pu2" +
                "VzzG97fAHs3MxZqFwc+72o8wYZ4P2AGQxG0dPhpGWRW5RBsCDXUdvr5cNe/sGoou33gNNF8Dd3YZsrK8Wn" +
                "oMFh3IAUxMkGZJ96pIhyO0JTMC2jQqVmWtngjxbofZJY05egmDZlGGQvw" +
                "85kg0UNLY322eW6XFzfHDdqAzKeEJWoWVLNJgG48h2LsNHzNV3cnZLvCfR6WtOq1dvYB" +
                "YPqEMKcHNKbLLVlKk2eSS4BK9hMBobeEplJVsE";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        //
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

// wait for the start button to be pressed.
        waitForStart();

        relicTrackables.activate();

        //leftDrive.setPower(0);
        //  rightDrive.setPower(0);

        int firsttime = 1;

        double speed = 0;

        int vuTries = 7;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && vuTries > 0) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuTries -= 1;
        }


        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.CENTER;
        }



                sleep(1000);
                servotest.setPosition(0.175);
                setDriveSpeed(0, 0);
                sleep(1000);
                if (colorsensor.red() > colorsensor.blue()) {

                        servoturn.setPosition(.3);
                        sleep(1000);
                        servotest.setPosition(.3);
                        sleep(1000);
                        servoturn.setPosition(0.5);
                        sleep(1000);
                    } else {
                        servoturn.setPosition(.7);
                        sleep(1000);
                        servotest.setPosition(.3);
                        sleep(1000);
                        servoturn.setPosition(0.5);
                        sleep(1000);
                    }

                servotest.setPosition(1);
                sleep(2000);

                setDriveSpeed(0.35, 0.35);
                sleep(1000);


                backmotorleft.setPower(.5);
                backmotorright.setPower(.5);
                sleep(600);

                backmotorleft.setPower(0);
                backmotorright.setPower(0);
                sleep(500);

                setDriveSpeed(-0.35, -0.35);
                sleep(1800);
        backmotorright.setPower(0);

                setDriveSpeed(0, 0);
                sleep(600);


                telemetry.addData("Color", "" + colorsensor2.red() + " / " + colorsensor2.green() + " / " + colorsensor2.blue());
                telemetry.update();
                colorsensor2.red();
                colorsensor2.blue();
                colorsensor2.green();


                while (isGray()) {
                    setDriveSpeed(speed, speed);
                    speed += 0.015;
                    if (speed > 1) {
                        speed = 1;
                    }
                    sleep(100);
                    telemetry.addData("Color", "gray");
                    telemetry.update();
                }

                telemetry.addData("Color", "not gray");
                telemetry.update();

                setDriveSpeed(0, 0);


                final double HEADING_EPSILON = 1.5;
                final double TURN_SPEED = 0.25;

if (vuMark == RelicRecoveryVuMark.RIGHT) {
    while (Math.abs(getHeading() + 85) > HEADING_EPSILON) ///Right
        {
        setDriveSpeed(TURN_SPEED, -TURN_SPEED);
        telemetry.addData("gyro", imu.getAngularOrientation().firstAngle);
        telemetry.update();
    }
}
                else if (vuMark == RelicRecoveryVuMark.LEFT) {
  setDriveSpeed(0.4,0.4);
  sleep(1000);

    while (Math.abs(getHeading() + 35 ) > HEADING_EPSILON)
                    {
                        setDriveSpeed(TURN_SPEED, -TURN_SPEED);
                        telemetry.addData("gyro", imu.getAngularOrientation().firstAngle);
                        telemetry.update();
                    }


                }

              else  {
                   while (Math.abs(getHeading() + 60 ) > HEADING_EPSILON)
                    {
                        setDriveSpeed(TURN_SPEED, -TURN_SPEED);
                        telemetry.addData("gyro", imu.getAngularOrientation().firstAngle);
                        telemetry.update();
                    }
                }

                setDriveSpeed(0, 0);
                setDriveSpeed(0.2, 0.2);
                sleep(1400);

                armDrive1.setPower(-0.8);
                armDrive2.setPower(-0.8);
                sleep(1000);

                armDrive1.setPower(0);
                armDrive2.setPower(0);
                sleep(1000);

                setDriveSpeed(-0.3, -0.3);
                sleep(320);

                setDriveSpeed(0, 0);
                sleep(1000);


            }




    boolean isGray() {
        final double COLOR_EPSILON = 60;

        if (Math.abs(colorsensor2.red() - colorsensor2.blue()) > COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(colorsensor2.red() - colorsensor2.green()) > COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(colorsensor2.blue() - colorsensor2.green()) > COLOR_EPSILON) {
            return false;
        }

        // must be gray
        return true;
    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    void setDriveSpeed(double left, double right) {
        leftDrive.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        rightDrive.setPower(-right);
//        rightDrive2.setPower(right * 0.9);
    }

    void setBackSpeed(double left, double right) {
        backmotorleft.setPower(-left);
//        leftDrive2.setPower(left * 0.9);
        backmotorright.setPower(right);
//        rightDrive2.setPower(right * 0.9);
    }
}
//});

//}
//        }
