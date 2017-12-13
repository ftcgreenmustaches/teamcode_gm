
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name="EXAMPLE", group="Linear Opmode")
public class EncoderExampleAjay extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor backmotorleft=null;




    //*******

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        backmotorleft = hardwareMap.get(DcMotor.class, "leftDrive");
        backmotorleft.setMode(DcMotor.RunMode.RESET_ENCODERS);
      while (backmotorleft.getCurrentPosition()!=0){
          backmotorleft.setMode(DcMotor.RunMode.RESET_ENCODERS);

      }
      backmotorleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      waitForStart();
      backmotorleft.setTargetPosition(1440);
    backmotorleft.setPower(0.5);

    backmotorleft.setPower(0);

    }}
