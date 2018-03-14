
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous(name = "TouchSensorTest", group = "Concept")
@Disabled
public class SensorTest extends LinearOpMode {
    CRServo relictwist;
    CRServo relicextend;
    private CRServo relicvertical;
    private DigitalChannel touchsensor2;
    private DcMotor armDrive1;
    private DcMotor armDrive2;

    public void runOpMode() {
        relictwist = hardwareMap.get(CRServo.class, "relictwist");
        relicextend = hardwareMap.get(CRServo.class, "relicextend");
        relicvertical = hardwareMap.get(CRServo.class, "relicvertical");
        touchsensor2 = hardwareMap.get(DigitalChannel.class, "touchsensor2");
        armDrive1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armDrive2 = hardwareMap.get(DcMotor.class, "armmotor2");
        touchsensor2.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        sleep(2000);
        while (opModeIsActive()) {
            if (touchsensor2.getState() == true) {
                armDrive2.setPower(0);
                armDrive1.setPower(0);
                sleep(2000);
                telemetry.addData("Digital Touch", "Is Not Pressed");
                telemetry.update();
            } else {
                armDrive1.setPower(0.5);
                armDrive2.setPower(0.5);
                sleep(2000);
                telemetry.addData("Digital Touch", "Is  Pressed");
                telemetry.update();
            }
        }
    }
}