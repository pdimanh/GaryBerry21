package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous


public class AutoParking8811 extends LinearOpMode {
    DcMotor FL, FR, BL, BR, LeftSucker, RightSucker;


    public void runOpMode(){

        FL=hardwareMap.dcMotor.get("FL");
        FR=hardwareMap.dcMotor.get("FR");
        BL=hardwareMap.dcMotor.get("BL");
        BR=hardwareMap.dcMotor.get("BR");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        LeftSucker=hardwareMap.dcMotor.get("LeftSucker");
        RightSucker=hardwareMap.dcMotor.get("RightSucker");

        waitForStart();

        strafeLeft(false,.6);
        sleep(1200);
        drive(0,0);
        sleep(1000);

        IntakeMotors();



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
    public void IntakeMotors(){
        RightSucker.setPower(-.85);
        LeftSucker.setPower(.85);
        sleep(1000);
        RightSucker.setPower(0);
        LeftSucker.setPower(0);
    }

}
