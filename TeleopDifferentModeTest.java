

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

import android.preference.PreferenceScreen;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.sql.Driver;

/**
 * Created by GM on 11/1/2017.
 * Program name : Teleop 2 Drivers progra
 * Purpose :
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the code for picking and dispensing .
 */


@TeleOp(name = "TELEOP ModeTest", group = "Linear Opmode")
public class TeleopDifferentModeTest extends LinearOpMode {

    // Declare OpMode members.
    private double servoPosition;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;
    private DcMotor armDrive3 = null;
    private DcMotor armDrive4 = null;
    //1 private Servo servoTest;
    private DcMotor backmotorleft = null;
    private DcMotor backmotorright = null;
    private DigitalChannel tsensor;
    CRServo relictwist;
    Servo relicvertical;
    CRServo relicextend;
    Servo relicpick;
    boolean extending = false;
    boolean twisting = false;
    boolean rising = false;
    boolean endgame = false;
    boolean pickrelic = false;
    boolean droprelic = false;
    boolean increments = false;
    private double servoDelta = 0.1;
    private double servoDelayTime = 0.01;

    //Servo relicpick;
    //private TouchSensor tsensor;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        this.servoPosition = 0.5;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "motorleft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorright");
        armDrive1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armDrive2 = hardwareMap.get(DcMotor.class, "armmotor2");
        //    servoTest=hardwareMap.get(Servo.class, "servotest");
        armDrive3 = hardwareMap.get(DcMotor.class, "armmotor3");
        armDrive4 = hardwareMap.get(DcMotor.class, "armmotor4");
        backmotorleft = hardwareMap.get(DcMotor.class, "backmotorleft");
        backmotorright = hardwareMap.get(DcMotor.class, "backmotorright");
        tsensor = hardwareMap.get(DigitalChannel.class, "touchsensor");
        relictwist = hardwareMap.get(CRServo.class, "relictwist");
        relicvertical = hardwareMap.get(Servo.class, "relicvertical");
        relicextend = hardwareMap.get(CRServo.class, "relicextend");
        relicpick = hardwareMap.get(Servo.class, "relicpick");
        //   relicpick=hardwareMap.get(Servo.class,"relickpick");

        int counter = 0;
        endgame = false;
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive2.setDirection(DcMotor.Direction.FORWARD);
        armDrive1.setDirection(DcMotor.Direction.FORWARD);
        armDrive3.setDirection(DcMotor.Direction.FORWARD);
        armDrive4.setDirection(DcMotor.Direction.FORWARD);
        backmotorright.setDirection(DcMotor.Direction.FORWARD);
        backmotorleft.setDirection(DcMotor.Direction.REVERSE);

        //determine the zeropowerbehavior
        boolean brake = true;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior;
        if (brake) {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        } else {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        }


        boolean extending = false;
        boolean twisting = false;
        boolean increments = false;
        boolean pickrelic = false;
        boolean droprelic = false;
        boolean vertical =false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad2.left_stick_button) {
                endgame = true;
            } else if (gamepad2.right_stick_button) {
                endgame = false;
            }

            if (!endgame) {
                // Setup a variable for each drive wheel to save power level for telemetry
                double leftPower;
                double rightPower;

                // Choose to drive using either Tank Mode, or POV Mode
                // Comment out the method that's not used.  The default below is POV.

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);


                // Tank aMode uses one stick to control each wheel.
                //- This requires no math, but it is hard to drive forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                //rightPower = -gamepad1.right_stick_y ;
                if (gamepad1.dpad_up) {
                    backmotorleft.setPower(1);
                    backmotorright.setPower(1);
                } else if (gamepad1.dpad_down) {
                    backmotorright.setPower(-1);
                    backmotorleft.setPower(-1);
                } else {
                    backmotorleft.setPower(0);
                    backmotorright.setPower(0);
                }

                if (gamepad2.right_bumper) {
                    armDrive1.setPower(0.7);
                    armDrive2.setPower(0.7);
                } else if (gamepad2.left_bumper) {
                    armDrive1.setPower(-0.7);
                    armDrive2.setPower(-0.7);
                } else if (gamepad2.b) {
                    armDrive1.setPower(-0.23);
                    armDrive2.setPower(-0.23);
                } else {
                    armDrive1.setPower(0);
                    armDrive2.setPower(0);
                }


                if (gamepad2.a) {
                    armDrive3.setPower(0.8);
                    armDrive4.setPower(-0.8);
                } else if (gamepad2.y && tsensor.getState())

                {
                    armDrive3.setPower(-0.8);
                    armDrive4.setPower(0.8);
                } else if (gamepad2.x) {
                    armDrive3.setPower(-0.7);
                    armDrive4.setPower(0.7);
                    sleep(1800);
                } else {
                    armDrive3.setPower(0);
                    armDrive4.setPower(0);

                }

                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();


            } else {   //must be endgame


                if (-gamepad1.left_stick_y > 0.01) {
                    relicextend.setPower(-gamepad1.left_stick_y * -0.99);
                    extending = true;
                } else if (gamepad1.left_stick_y > 0.1) {
                    relicextend.setPower(gamepad1.left_stick_y * 0.79);
                    extending = true;
                } else if (extending) {
                    extending = false;
                    relicextend.getController().pwmDisable();
                }



          /*      if (gamepad2.left_stick_y > 0.01) {
                    relicextend.setPower(gamepad2.left_stick_y * .79);
                    extending = true;
                } else if (-gamepad2.left_stick_y > 0.01) {
                    relicextend.setPower(-gamepad2.left_stick_y * -.79);
                    extending = true;
                } else if (extending) {
                    extending = false;
                    relictwist.getController().pwmDisable();
                }
*/
                if (gamepad1.right_trigger > 0.01) {
                    relictwist.setPower(gamepad1.right_trigger * .79);
                    twisting = true;
                } else if (gamepad1.left_trigger > 0.01) {
                    relictwist.setPower(gamepad1.left_trigger * -.79);
                    twisting = true;
                } else if (twisting) {
                    twisting = false;
                    relictwist.getController().pwmDisable();
                }

              /*  if (gamepad1.dpad_up) {
                    relicvertical.setPower(-0.5);
                    vertical = true;
                } else if (gamepad1.dpad_down) {
                    relicvertical.setPower(0.5);
                    vertical = true;
                }else if (vertical) {
                    vertical = false;
                    relictwist.getController().pwmDisable();
                }
*/
              if (gamepad1.dpad_up){
                  relicvertical.setPosition(.9);
              }

              if (gamepad1.dpad_down){
                  relicvertical.setPosition(.1);
              }



                //   if (gamepad2.y) {
                //     relicvertical.setPosition(.5);
//               /     servoPosition += servoDelta;
                // clip the position values so that they never exceed 0..1
                //                  servoPosition = Range.clip(servoPosition, 0, 1);
                ////                relicvertical.setPosition(servoPosition);
//
                //                   relicvertical.setPosition(relicvertical.getPosition() - .05);
                //                 increments = true;

  //              if (increments) {
    //                increments = false;
      //          }


                if (gamepad1.left_bumper) {
                    relicpick.setPosition(.9);
                } else if (gamepad1.right_bumper) {
                    relicpick.setPosition(.2);
                }

                if (gamepad1.a) {
                    droprelic = true;
                    relicextend.setPower(.79);
                    sleep(400);
                    relicvertical.setPosition(-.5);
                    relicextend.setPower(0.79);
                    sleep(400);
                    relictwist.setPower(.79);
                    sleep(500);
                }
                    // Setup a variable for each drive wheel to save power level for telemetry
                    double leftPower;
                    double rightPower;

                    // Choose to drive using either Tank Mode, or POV Mode
                    // Comment out the method that's not used.  The default below is POV.

                    // POV Mode uses left stick to go forward, and right stick to turn.
                    // - This uses basic math to combine motions and is easier to drive straight.
                    double drive = -gamepad2.left_stick_y;
                    double turn = gamepad2.right_stick_x;

                    leftPower = Range.clip(drive + turn, -1.0, 1.0);
                    rightPower = Range.clip(drive - turn, -1.0, 1.0);


                    // Tank aMode uses one stick to control each wheel.
                    //- This requires no math, but it is hard to drive forward slowly and keep straight.
                    // leftPower  = -gamepad1.left_stick_y ;
                    //rightPower = -gamepad1.right_stick_y ;
                    if (gamepad2.dpad_up) {
                        backmotorleft.setPower(1);
                        backmotorright.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        backmotorright.setPower(-1);
                        backmotorleft.setPower(-1);
                    } else {
                        backmotorleft.setPower(0);
                        backmotorright.setPower(0);
                    }

                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }


        }
    }
}
















