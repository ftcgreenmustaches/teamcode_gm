package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous ;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name = "Armtest", group = "Concept")
public class JewelArmTest extends LinearOpMode {
    CRServo relicexend;
    CRServo relictwist;
    Servo relicpick;

    public void runOpMode() {
        relictwist = hardwareMap.get(CRServo.class, "relictwist");
        relicexend = hardwareMap.get(CRServo.class, "relicextend");
        relicpick = hardwareMap.get(Servo.class, "relicpick");
        waitForStart();
        while (opModeIsActive()) {


            relicexend.setPower(.8);
            sleep(300);
            relicexend.setPower(-.8);
            sleep(300);

            relicexend.setPower(.5);
            sleep(5000);

            if (gamepad1.dpad_up) {
                relicexend.setPower(-0.99);
            } else {
                relicexend.setPower(0.5);
            }


        }
    }
}