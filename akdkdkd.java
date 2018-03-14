

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

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by GM on 11/1/2017.
 * Program name : Teleop 2 Drivers progra
 * Purpose :
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the code for picking and dispensing .
 *
 */


@TeleOp(name="lqwehfb", group="Linear Opmode")
@Disabled
public class akdkdkd extends LinearOpMode {

    // Declare OpMode members.
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
    CRServo relicvertical;
    CRServo relicextend;
    CRServo relicpick;

    //Servo relicpick;
    //private TouchSensor tsensor;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        tsensor = hardwareMap.get(DigitalChannel.class, "digitaltouch");
        relictwist = hardwareMap.get(CRServo.class, "relictwist");
        relicvertical = hardwareMap.get(CRServo.class, "relicvertical");
        relicextend = hardwareMap.get(CRServo.class, "relicextend");
        relicpick = hardwareMap.get(CRServo.class, "relicpick");
        //   relicpick=hardwareMap.get(Servo.class,"relickpick");

        int counter = 0;

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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*if (gamepad1.b) {
                relicextend.setPower(-1);
            }
            else {
                relicextend.setPower(.6);

                if (gamepad1.a) {
                    relicextend.setPower(1);
                } else {
                    relicextend.setPower(.6);


                    //else if (gamepad2.dpad_up) {
                    //  relicextend.setPower(1);
                    // }else {
                    //     relicextend.setPower(0.5);
                    // }
*/
            if (gamepad1.x) {
                relicextend.setPower(0.6);
            } else {
                relicextend.setPower(0);
                sleep(100);
            }
            if (gamepad1.a) {
                relicextend.setPower(-0.8);
            } else {
                relicextend.setPower(0);
                sleep(100);
            }

        }
    }
}