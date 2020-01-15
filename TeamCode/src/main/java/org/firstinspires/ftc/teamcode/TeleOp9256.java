package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TeleOp9256 extends OpMode {

    DcMotor FL, FR, BL, BR;
    Servo josh,daisy;
    private final double JOSH_UP=.12;
    private final double JOSH_DOWN=.5;
    private final double DAISY_UP=.86;
    private final double DAISY_DOWN=.5;

    Servo bryan;
    DcMotor Lefty, Righty;
    Servo Sayra;
    private final double SAYRA_UP=.54;
    private final double SAYRA_DOWN=.17;

    public void init(){
        FL=hardwareMap.dcMotor.get("FL");
        FR=hardwareMap.dcMotor.get("FR");
        BL=hardwareMap.dcMotor.get("BL");
        BR=hardwareMap.dcMotor.get("BR");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        Lefty=hardwareMap.dcMotor.get("Lefty");
        Righty=hardwareMap.dcMotor.get("Righty");
        Righty.setDirection(DcMotor.Direction.REVERSE);

        Lefty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Righty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        josh=hardwareMap.servo.get("josh");
        josh.setPosition(JOSH_UP);
        daisy=hardwareMap.servo.get("daisy");
        daisy.setPosition(DAISY_UP);
        bryan=hardwareMap.servo.get("bryan");
        bryan.setPosition(.5);
        Sayra=hardwareMap.servo.get("Sayra");
        Sayra.setPosition(SAYRA_UP);




    }
    public void loop(){
        driveMecanum();
        grabFoundation();
        gayLyft();
        grabSayra();
        extendBryan();

    }
    public void driveMecanum(){
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        FR.setPower(drive+turn+strafe);
        FL.setPower(drive-turn-strafe);
        BR.setPower(drive+turn-strafe);
        BL.setPower(drive-turn+strafe);

    }
    public void grabFoundation(){
        if(gamepad1.right_bumper){
            daisy.setPosition(DAISY_DOWN);
            josh.setPosition(JOSH_DOWN);
        }
        if(gamepad1.left_bumper){
            daisy.setPosition(DAISY_UP);
            josh.setPosition(JOSH_UP);

        }


    }
    public void gayLyft(){
        if(gamepad2.dpad_up){
            Lefty.setPower(1);
            Righty.setPower(1);
        }else if(gamepad2.dpad_down){
            Lefty.setPower(-1);
            Righty.setPower(-1);

        }else{
            Lefty.setPower(0);
            Righty.setPower(0);
        }
    }
    public void grabSayra(){
        if(gamepad1.y){
            Sayra.setPosition(SAYRA_UP);

        }
        if(gamepad1.x){
            Sayra.setPosition(SAYRA_DOWN);

        }



    }
    public void extendBryan(){
        if(gamepad2.right_trigger>.5){
            bryan.setPosition(.3);
        }else if(gamepad2.left_trigger>.5){
            bryan.setPosition(.7);
        }else{
            bryan.setPosition(.5);
        }


    }





}
