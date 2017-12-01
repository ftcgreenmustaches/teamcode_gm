package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

/**
 * Created by aparikh1 on 11/1/2017.
 */

@Autonomous(name = "REDfar/Vuforia", group = "Linear Opmode")
//@Disabled
public class REDfarwithVuforia extends LinearOpMode {

    public static final double JEWEL_SPEED = 0.35;
    public static final int JEWEL_TIME = 500;
    public static final double CR_DOWN = -0.75;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    //private DcMotor armDrive1 = null;
    //private DcMotor armDrive2 = null;
    private Servo servotest;
    private ColorSensor colorSensor = null;
    private DcMotor backmotorleft = null;
    private DcMotor backmotorright = null;
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "motorleft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorright");
        // armDrive1 = hardwareMap.get(DcMotor.class, "armmotor1");
        //armDrive2 = hardwareMap.get(DcMotor.class, "armmotor2");
        servotest = hardwareMap.get(Servo.class, "servotest");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        backmotorleft = hardwareMap.get(DcMotor.class, "backmotorleft");
        backmotorright = hardwareMap.get(DcMotor.class, "backmotorright");

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();


        leftDrive.setDirection(Direction.FORWARD);
        rightDrive.setDirection(Direction.REVERSE);
        //armDrive2.setDirection(DcMotor.Direction.FORWARD);
        //armDrive1.setDirection(DcMotor.Direction.FORWARD);
        backmotorleft.setDirection(Direction.REVERSE);
        backmotorright.setDirection(Direction.FORWARD);


        waitForStart();
        runtime.reset();

        int vuTries = 10;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && vuTries > 0) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuTries -= 1;
        }

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.CENTER;
        }

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
            sleep(JEWEL_TIME * 5);
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

        rightDrive.setPower(-0.35);
        leftDrive.setPower(-0.35);
        sleep(1000);

        setDriveSpeed(.25, .25);
        sleep(400);
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            rightDrive.setPower(0);
            leftDrive.setPower(-.7);
            sleep(701);

        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            rightDrive.setPower(.1);
            leftDrive.setPower(1);
            sleep(601);

        } else {
            rightDrive.setPower(1);
            leftDrive.setPower(-.25);
            sleep(301);

        }
//  commented by ajay
    //    rightDrive.setPower(1);
     //   leftDrive.setPower(-.25);
      //  sleep(301);


        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.update();

        telemetry.update();

    }


    void setDriveSpeed(double left, double right) {
        leftDrive.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        rightDrive.setPower(right);
//        rightDrive2.setPower(right * 0.9);
    }

    void setBackSpeed(double left, double right) {
        backmotorleft.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        backmotorright.setPower(right);
//        rightDrive2.setPower(right * 0.9);
        telemetry.update();
    }

}








































