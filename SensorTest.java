
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous ;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
@Autonomous(name = "ConceptScanServo", group = "Concept")
public class SensorTest extends LinearOpMode {
    CRServo relictwist;
CRServo relicextend;
    private CRServo relicvertical;

    public void runOpMode(){
        relictwist=hardwareMap.get(CRServo.class,"relictwist");
            relicextend=hardwareMap.get(CRServo.class,"relicextend");
relicvertical=hardwareMap.get(CRServo.class,"relicvertical");
            waitForStart();
      relicextend.setPower(-0.5);
      relicvertical.setPower(-0.5);
sleep(4000);
        relicextend.setPower(0.5);
        relicvertical.setPower(0.5);
        sleep(5000);


        }
    }
