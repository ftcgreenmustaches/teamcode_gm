package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by GM on 11/1/2017.
 * Program name : Red Close
 * Purpose : This Autonomous program is from Red Close
 */

@Autonomous(name="REDclose", group="Linear Opmode")
//@Disabled
public class REDclose extends LinearOpMode {

    public static final double JEWEL_SPEED = 0.35;
    public static final int JEWEL_TIME = 500;
    public static final double CR_DOWN = -0.75;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;
    private Servo servotest;
    private ColorSensor colorSensor=null;
    private DcMotor backmotorleft=null;
    private DcMotor backmotorright=null;
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
        backmotorleft = hardwareMap.get(DcMotor.class, "backmotorleft");
        backmotorright = hardwareMap.get(DcMotor.class, "backmotorright");



        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive2.setDirection(DcMotor.Direction.FORWARD);
        armDrive1.setDirection(DcMotor.Direction.FORWARD);
        backmotorleft.setDirection(Direction.REVERSE);
        backmotorright.setDirection(Direction.FORWARD);


        waitForStart();
        runtime.reset();



        servotest.setPosition(CR_DOWN);

        //leftDrive.setPower(0);
        //  rightDrive.setPower(0);
        setDriveSpeed(0, 0);
        sleep(2000);

        boolean forward = true;
        if (colorSensor.red() > colorSensor.blue()) {
            setDriveSpeed(JEWEL_SPEED, JEWEL_SPEED);
        } else {
            forward = false;
            setDriveSpeed(-JEWEL_SPEED, -JEWEL_SPEED);
        }

        sleep(JEWEL_TIME);
        servotest.setPosition(1);
        setDriveSpeed(0, 0);
        // leftDrive.setPower(0);
        //rightDrive.setPower(0);

        //leftDrive.setPower(JEWEL_SPEED);
        //rightDrive.setPower(JEWEL_SPEED);
        setDriveSpeed(JEWEL_SPEED, JEWEL_SPEED);
        if (forward) {
            sleep(JEWEL_TIME * 2);
        } else {
            sleep(JEWEL_TIME * 4);
   //                           PREVIOUSLY MULTIPLIED BY 5
        }
        //  leftDrive.setPower(0);
        //rightDrive.setPower(0);
        setDriveSpeed(0, 0);
        sleep(1000);

        backmotorleft.setPower(.5);
        backmotorright.setPower(.5);
        sleep(300);

        setDriveSpeed(0, 0);
        sleep(1000);

        final double backpowerratio = 0.7;
        rightDrive.setPower(-0.35);
        leftDrive.setPower(-0.35);
        backmotorleft.setPower(-0.35*backpowerratio);
        backmotorright.setPower(-.35*backpowerratio);
        sleep(1000);


        rightDrive.setPower(0.25);
        leftDrive.setPower(0.25);
        backmotorleft.setPower(.25*backpowerratio);
        backmotorright.setPower(.25*backpowerratio);
        sleep(300);
  //Changed 1117
//        setDriveSpeed(.25, .25);
  //      sleep(400);

// changed fro -.5 to .7
        rightDrive.setPower(-.8);
        leftDrive.setPower(1);
        sleep(501);
        //setDriveSpeed(0, 0);
        //sleep(500);

         setDriveSpeed(.3,.3);
         sleep(500);

        armDrive1.setPower(-0.8);
        armDrive2.setPower(-0.8);
        sleep(1000);


        setDriveSpeed(0, 0);
        sleep(1000);

        setDriveSpeed(-0.5,-0.5);
        sleep(100);

        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.update();
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










