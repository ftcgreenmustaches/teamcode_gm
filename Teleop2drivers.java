

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


@TeleOp(name="TELEOPMAIN 2 Drivers", group="Linear Opmode")
public class Teleop2drivers extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 =null;
    private DcMotor armDrive3=null;
    private DcMotor armDrive4=null;
    private Servo servoTest;
    private DcMotor backmotorleft=null;
    private DcMotor backmotorright=null;
    private DigitalChannel tsensor;

    //private TouchSensor tsensor;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "motorleft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorright");
        armDrive1=hardwareMap.get(DcMotor.class, "armmotor1");
        armDrive2=hardwareMap.get(DcMotor.class, "armmotor2");
        servoTest=hardwareMap.get(Servo.class, "servotest");
        armDrive3=hardwareMap.get(DcMotor.class, "armmotor3");
        armDrive4=hardwareMap.get(DcMotor.class, "armmotor4");
        backmotorleft=hardwareMap.get(DcMotor.class,"backmotorleft");
        backmotorright=hardwareMap.get(DcMotor.class, "backmotorright");
        tsensor = hardwareMap.get(DigitalChannel.class, "digitaltouch");
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
        boolean brake=true;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior;
        if(brake) {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        } else {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        }



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
     
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0);


             // Tank aMode uses one stick to control each wheel.
            //- This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            //rightPower = -gamepad1.right_stick_y ;

            if (gamepad2.right_bumper){
                armDrive1.setPower(0.7);
                armDrive2.setPower(0.7);
            }else if (gamepad2.left_bumper){
                armDrive1.setPower(-0.7);
                armDrive2.setPower(-0.7);
            }else{
                armDrive1.setPower(0);
                armDrive2.setPower(0);
            }


            if (gamepad2.a){
                armDrive3.setPower(0.8);
                armDrive4.setPower(-0.8);
            }else if (gamepad2.y && tsensor.getState())

            {  armDrive3.setPower(-0.8);
                armDrive4.setPower(0.8);
            }else{
                armDrive3.setPower(0);
                armDrive4.setPower(0);

            }

            if (gamepad1.dpad_up){
                backmotorleft.setPower(1);
                backmotorright.setPower(1);
            }
            else if (gamepad1.dpad_down){
                backmotorright.setPower(-1);
                backmotorleft.setPower(-1);
            }
            else{
                backmotorleft.setPower(0);
                backmotorright.setPower(0);
            }



            if (gamepad1.x){
                servoTest.setPosition(1);
            }


        if (gamepad1.right_bumper) {
            leftDrive.setPower(.35);
        rightDrive.setPower(-0.35);
            }else if (gamepad1.left_bumper) {
        leftDrive.setPower(-0.35);
        rightDrive.setPower(0.35);
        }  else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }


            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
              // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}














