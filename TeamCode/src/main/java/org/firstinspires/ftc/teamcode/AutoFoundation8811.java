package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class AutoFoundation8811 extends LinearOpMode {

    DcMotor FL, FR, BL, BR;
    Servo FoundationGrabber;

    public void runOpMode(){

        FL=hardwareMap.dcMotor.get("FL");
        FR=hardwareMap.dcMotor.get("FR");
        BL=hardwareMap.dcMotor.get("BL");
        BR=hardwareMap.dcMotor.get("BR");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();
    }

    public void drive(double leftpower, double rightpower) {
        FL.setPower(leftpower);
        FR.setPower(rightpower);
        BL.setPower(leftpower);
        BR.setPower(rightpower);

    }

    public void strafeLeft(boolean left, double power) {
        if (left == true) {
            FL.setPower(-power);
            FR.setPower(power);
            BL.setPower(power);
            BR.setPower(-power);
        } else {
            FL.setPower(power);
            FR.setPower(-power);
            BL.setPower(-power);
            BR.setPower(power);
        }
    }
}
