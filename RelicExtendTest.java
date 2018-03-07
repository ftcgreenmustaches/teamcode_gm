package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Relicextendtest", group = "Concept")
public class RelicExtendTest extends LinearOpMode {
    CRServo relicextend;
    CRServo relictwist;

    boolean extending = false;
    boolean twisting = false;

    public void runOpMode() {

        relicextend = hardwareMap.get(CRServo.class, "relicextend");
        relictwist = hardwareMap.get(CRServo.class, "relictwist");
        extending = false;
        twisting = false;
        waitForStart();


        while (opModeIsActive()) {


            if (gamepad1.dpad_up) {
                relicextend.setPower(-0.99);
                extending = true;
            } else if (gamepad1.dpad_down) {
                relicextend.setPower(0.79);
                extending = true;
            } else if (extending) {
                extending = false;
                relicextend.getController().pwmDisable();
            }

            if (gamepad1.dpad_right) {
                telemetry.addData("twist", "right");
                relictwist.setPower(-.79);
                twisting = true;
            } else if (gamepad1.dpad_left) {
                telemetry.addData("twist", "left");
                    relictwist.setPower(0.79);
                    twisting = true;
            } else if (twisting) {
                telemetry.addData("twist", "stop");
                twisting = false;
                relictwist.getController().pwmDisable();
            }
            telemetry.update();
        }
    }

}


