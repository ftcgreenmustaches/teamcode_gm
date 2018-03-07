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
    CRServo relicvertical;
    Servo relicpick;
    public void runOpMode() {
        relictwist = hardwareMap.get(CRServo.class, "relictwist");
        relicexend = hardwareMap.get(CRServo.class, "relicextend");
        relicvertical = hardwareMap.get(CRServo.class, "relicvertical");
        relicpick=hardwareMap.get(Servo.class,"relicpick");
        waitForStart();
        while(opModeIsActive()){

        relicpick.setPosition(.8);
        relicvertical.setPower(-0.8);
        sleep(500);
        relicvertical.setPower(0.5);
/*
            if (gamepad1.dpad_up) {
                relicexend.setPower(-0.99);
            }
     if (gamepad1.dpad_down) {
                relicexend.setPower(0.79);
            }

            if (gamepad1.y)
                {
                //  servo.setPower(-0.3);
                relicexend.getController().pwmDisable();
            }
                if (gamepad1.dpad_right) {
//this takes the arm up
                relicvertical.setPower(-0.8);
                sleep(100);
                relicvertical.setPower(0.5);

                    relicpick.setPosition(.8);

                } if (gamepad1.dpad_left) {
//This takes the arm back
                relicvertical.setPower(0.8);
                relicpick.setPosition(.8);
                }


            if (gamepad1.x){
               //this position the arm is up
                      relicpick.setPosition(.2);

            } if (gamepad1.b){
                //this position the arm is down grabs the relic
                      relicpick.setPosition(.8);
                }

            if (gamepad1.a){

                relicvertical.getController().pwmDisable();
                relicpick.setPosition(.8);
            }
             /*
                if (gamepad1.x){
         //       relicpick.setPosition(.2);
                relictwist.setPower(-0.7);

            } if (gamepad1.b){
           //     relicpick.setPosition(.8);
                relictwist.setPower(0.79);
            }


            if (gamepad1.a){
                //     relicpick.setPosition(.8);
                relictwist.getController().pwmDisable();
            }
 */

            }
        }
    }
