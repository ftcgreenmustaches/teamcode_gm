
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="EncoderExample", group="Linear Opmode")
@Disabled
public class EncoderExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }
        private DcMotor backmotorleft = null;

        private DcMotor backmotorright = null;
        public static final double CountsPerRev = 134.4;    // Andymark NeveRest 40
        public static final double GearRatio = 16 / 18;
        public static final double WheelDiameterInches = 4.0;     // For figuring circumference
        public static final double CountsPerInch = ((CountsPerRev / (WheelDiameterInches * 3.1415)) * GearRatio);


    public void EncoderDrive(double speed, double leftInches, double rightInches, double AccelerationInches, int Direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        int RightPosition;
        int LeftPosition;
        double LeftPower;
        double RightPower;

        // Resets encoders to 0
        backmotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backmotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Checks to make sure that encoders are reset.
        while (backmotorleft.getCurrentPosition() > 1 && backmotorright.getCurrentPosition() > 1) {
            sleep(25);
        }

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = backmotorleft.getCurrentPosition() + (int) (leftInches * CountsPerInch);
            NewRightTarget = backmotorright.getCurrentPosition() + (int) (rightInches * CountsPerInch);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            RightPosition = backmotorright.getCurrentPosition();
            LeftPosition = backmotorleft.getCurrentPosition();
            // Gives the encoders the target.
            backmotorleft.setTargetPosition(NewLeftTarget);
            backmotorright.setTargetPosition(NewRightTarget);

            backmotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backmotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while (backmotorleft.getCurrentPosition() > 1) {
                sleep(15);
            }


            // Turn On RUN_TO_POSITION
            backmotorleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backmotorright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            waitForStart();
        //    EncoderDrive(.5, 10, 10, 4, Forward);

            // This gets where the motor encoders will be at full position when it will be at full speed.
            double LeftEncoderPositionAtFullSpeed = ((AccelerationInches * (CountsPerInch)) + LeftPosition);
            double RightEncoderPositionAtFullSpeed = ((AccelerationInches * (CountsPerInch)) + RightPosition);
            boolean Running = true;


            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (backmotorleft.isBusy() && backmotorright.isBusy() && Running && opModeIsActive()) {
                // While encoders are not at position
                if (((Math.abs(speed)) - (Math.abs(backmotorleft.getPower()))) > .05) {
                    // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                    LeftPower = (Range.clip(Math.abs((backmotorleft.getCurrentPosition()) / (LeftEncoderPositionAtFullSpeed)), .15, speed));
                    RightPower = (Range.clip(Math.abs((backmotorright.getCurrentPosition()) / (RightEncoderPositionAtFullSpeed)), .15, speed));

                    backmotorleft.setPower(LeftPower * Direction);
                    backmotorright.setPower(RightPower * Direction);

               //    telemetry.addData("In Accel loop CM", +Distance.getDistance(DistanceUnit.CM));
                   telemetry.addData("Accelerating", RightEncoderPositionAtFullSpeed);
                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", backmotorleft.getCurrentPosition(), backmotorright.getCurrentPosition());
                //    telemetry.addData("Sections Complete:", +SectionsCompleted);
                    telemetry.update();
                } else if (Math.abs(NewLeftTarget) - Math.abs(backmotorleft.getCurrentPosition()) < -1) {
                    Running = false;
                } else {
                    // Multiplies the speed desired by the direction, which has a value of either 1, or -1, and allows for backwards driving with the ramp up function
                    backmotorleft.setPower((speed * Direction));
                    backmotorright.setPower((speed * Direction));

                  //  telemetry.addData("In Reg loop CM", +Distance.getDistance(DistanceUnit.CM));
                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", backmotorleft.getCurrentPosition(), backmotorright.getCurrentPosition());
                    //telemetry.addData("Sections Complete:", +SectionsCompleted);
                    telemetry.update();
                }

                // Display information for the driver.


            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            backmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backmotorright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            backmotorleft.setPower(0);
            backmotorright.setPower(0);

        }
    }
}

