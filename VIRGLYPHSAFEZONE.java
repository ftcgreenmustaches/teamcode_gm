package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by aparikh1 on 11/1/2017.
 */

@Autonomous(name="VIRGLYPHREDSAFEZONE", group="Linear Opmode")
//@Disabled
public class VIRGLYPHSAFEZONE extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;
    private Servo servotest;
private ColorSensor colorSensor=null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "motorleft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorright");
        armDrive1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armDrive2 = hardwareMap.get(DcMotor.class, "armmotor2");
        servotest = hardwareMap.get(Servo.class, "servotest");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive2.setDirection(DcMotor.Direction.FORWARD);
        armDrive1.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();
        runtime.reset();

        servotest.setPosition(-1);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(2000);

        if (colorSensor.red()>colorSensor.blue()){
        rightDrive.setPower(0.5);
        leftDrive.setPower(0.5);
        sleep(400);

            servotest.setPosition(0);
        }
 

          else if (colorSensor.red()<colorSensor.blue()) {
            rightDrive.setPower(-0.3);
            leftDrive.setPower(-0.3);
             sleep(400);
             servotest.setPosition(0);}
          else {
            rightDrive.setPower(0);
            leftDrive.setPower(0);
        }

        servotest.setPosition(0);


        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        sleep(1000);

        leftDrive.setPower(0);
        rightDrive.setPower(0);sleep(1000);

        leftDrive.setPower(-.5);
        rightDrive.setPower(-.5);
        sleep(300);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(1000);

        leftDrive.setPower(.5);
        rightDrive.setPower(.5);
        sleep(250);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(1000);

        leftDrive.setPower(0.3);
        rightDrive.setPower(0);
        sleep(1600);

        rightDrive.setPower(0);
        leftDrive.setPower(0);
        sleep(1000);

        rightDrive.setPower(-.3);
        leftDrive.setPower(.3);
        sleep(1600);

        rightDrive.setPower(0);
        leftDrive.setPower(0);
        sleep(1000);

        rightDrive.setPower(-.31);
        leftDrive.setPower(-.3);
        sleep(1000);

        rightDrive.setPower(0);
        leftDrive.setPower(0);
        sleep(1000);

        rightDrive.setPower(.3);
        leftDrive.setPower(.3);
        sleep(1000);

        rightDrive.setPower(-.3);
        leftDrive.setPower(-.3);
        sleep(1000);


        armDrive1.setPower(0.7);
        armDrive2.setPower(0.7);
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        sleep(800);

    }
}
























