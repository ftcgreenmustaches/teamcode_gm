package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by aparikh1 on 11/1/2017.
 */

@Autonomous(name="GYROTURN90", group="Linear Opmode")
public class GyroTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    //private DcMotor armDrive1 = null;
    //private DcMotor armDrive2 = null;
    private Servo servotest;
    private ColorSensor colorSensor = null;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "motorleft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorright");
      //  armDrive1 = hardwareMap.get(DcMotor.class, "armmotor1");
        //armDrive2 = hardwareMap.get(DcMotor.class, "armmotor2");
        servotest = hardwareMap.get(Servo.class, "servotest");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //armDrive2.setDirection(DcMotor.Direction.FORWARD);
        //armDrive1.setDirection(DcMotor.Direction.FORWARD);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;
        double stopAngle = startAngle - 90;

        telemetry.addData("stopAngle", stopAngle);
        while (Math.abs(angles.firstAngle - stopAngle) < 10) {
            setDriveSpeed(-0.3, 0.3);
            sleep(50);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("stopAngle", stopAngle);
            telemetry.addData("heading", "" + angles.firstAngle + "/" + angles.secondAngle + "/" + angles.thirdAngle);
            telemetry.addData("delta", Math.abs(angles.firstAngle - stopAngle));
            telemetry.update();
        }

        while (true) {
            sleep(50);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("stopAngle", stopAngle);
            telemetry.addData("heading", "" + angles.firstAngle + "/" + angles.secondAngle + "/" + angles.thirdAngle);
            telemetry.addData("delta", Math.abs(angles.firstAngle - stopAngle));
            telemetry.addData("done", "");
            telemetry.update();
        }

    }
    void setDriveSpeed(double left, double right) {
        leftDrive.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        rightDrive.setPower(right);
//        rightDrive2.setPower(right * 0.9);
    }
}








































