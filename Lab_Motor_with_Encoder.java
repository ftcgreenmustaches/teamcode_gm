
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains test code to operate motor with a limit based on encoder values
 */

@TeleOp(name = "Motor & Encoder Test", group = "Linear Opmode")

public class Lab_Motor_with_Encoder extends LinearOpMode {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;


    long StartPosition;
    long MaxPosition;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftDrive = hardwareMap.dcMotor.get("motorleft");
        rightDrive = hardwareMap.dcMotor.get("motorright");

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StartPosition = leftDrive.getCurrentPosition();
     //   MaxPosition = StartPosition + 1680;//HEX
        MaxPosition=StartPosition+1440;//TETRIX
        telemetry.addData("Initial Position", StartPosition);
        telemetry.update();


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //motorLift.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //motorLift.setPower(gamepad2.left_stick_y);
            if (gamepad2.left_stick_y > 0 && leftDrive.getCurrentPosition() <= MaxPosition) {
                leftDrive.setPower(gamepad2.left_stick_y);
                rightDrive.setPower(gamepad2.left_stick_y);
            } else {
                if (gamepad2.left_stick_y < 0 && leftDrive.getCurrentPosition() >= StartPosition) {
                    leftDrive.setPower(gamepad2.left_stick_y);
                    rightDrive.setPower(gamepad2.left_stick_y);
                } else {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                }
            }
        }
    }
}