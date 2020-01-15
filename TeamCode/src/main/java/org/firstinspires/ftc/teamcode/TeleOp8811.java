package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp 8811")
public class TeleOp8811 extends OpMode {
    DcMotor FL, FR, BL, BR, LeftLyft, RightLyft, LeftSucker, RightSucker;
    Servo Joe, Extender,LeftGrabber,RightGrabber,FoundationGrabber;
    private int LyftTarget=0;
private double initial_positon_Joe=.5;
private double down_position_Joe=.95;
private boolean invertedDrive=true;

    public void init(){
        FL=hardwareMap.dcMotor.get("FL");
        FR=hardwareMap.dcMotor.get("FR");
        BL=hardwareMap.dcMotor.get("BL");
        BR=hardwareMap.dcMotor.get("BR");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);


        LeftLyft=hardwareMap.dcMotor.get("LeftLyft");
        RightLyft=hardwareMap.dcMotor.get("RightLyft");
        LeftLyft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLyft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLyft.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftLyft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLyft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLyft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLyft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        LeftSucker=hardwareMap.dcMotor.get("LeftSucker");
        RightSucker=hardwareMap.dcMotor.get("RightSucker");

        Joe=hardwareMap.servo.get("Joe");
        Joe.setPosition(initial_positon_Joe);

        Extender=hardwareMap.servo.get("Extender");
        Extender.setPosition((.5));

        LeftGrabber=hardwareMap.servo.get("LeftGrabber");
        RightGrabber=hardwareMap.servo.get("RightGrabber");
        LeftGrabber.setPosition(.38);
        RightGrabber.setPosition(.58);

        FoundationGrabber=hardwareMap.servo.get("FoundationGrabber");
        FoundationGrabber.setPosition(.3);

    }

    public void loop(){
        if(gamepad1.left_bumper){
            invertedDrive=true;
        }
        if(gamepad1.right_bumper){
            invertedDrive=false;
        }

        if(invertedDrive==true){
            invertedMecanum();

        }else{
            driveMecanum();
        }
        riseLyft();
        intakeMotors();
        positionBlocks();
        extendGrabber();
        grabBlock();
        grabFoundation();
        

    }
    public void positionBlocks(){
        if(gamepad1.b){
            Joe.setPosition(down_position_Joe);
        }else{
            Joe.setPosition(initial_positon_Joe);

        }
    }

    public void driveMecanum(){
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if(Math.abs(drive) > .1){
            drive = drive;
        } else{
            drive = 0;
        }


        FR.setPower(drive+turn-strafe);
        FL.setPower(drive-turn+strafe);
        BR.setPower(drive+turn+strafe);
        BL.setPower(drive-turn-strafe);

    }
    public void invertedMecanum(){

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        if(Math.abs(drive) > .1){
            drive = drive;
        } else{
            drive = 0;
        }

        FR.setPower(drive+turn+strafe);
        FL.setPower(drive-turn-strafe);
        BR.setPower(drive+turn-strafe);
        BL.setPower(drive-turn+strafe);

    }


    public void riseLyft(){
        telemetry.addData("EncoderValue",LeftLyft.getCurrentPosition());
        telemetry.addData("EncoderValue",RightLyft.getCurrentPosition());
        telemetry.addData("LyftTarget",LyftTarget);
        if (gamepad2.dpad_up){
            //LyftTarget=LyftTarget+10;

            LyftTarget = 5500;

        }else if (gamepad2.dpad_down){
            //LyftTarget=LyftTarget-10;

            LyftTarget = 0;

        }



        if(gamepad2.dpad_left){

            LyftTarget=LyftTarget-10;
        }

        if(gamepad2.dpad_right){
            LyftTarget=LyftTarget+10;

        }

        if (gamepad2.a){
            if (LyftTarget>LeftLyft.getCurrentPosition()){

                if(Math.abs(LyftTarget-LeftLyft.getCurrentPosition())>50){









                    LeftLyft.setPower(1);
                }else{
                    LeftLyft.setPower(0);
                }



            }else if (LyftTarget<LeftLyft.getCurrentPosition()){
               if(Math.abs(LyftTarget-LeftLyft.getCurrentPosition())>50){

                   if (Math.abs(LyftTarget - LeftLyft.getCurrentPosition()) > 1000) {


                       LeftLyft.setPower(-.7);
                   } else{

                       LeftLyft.setPower(-.3);
                   }


               }else{
                 LeftLyft.setPower(0);
               }


            }else{
                LeftLyft.setPower(0);
            }

            if (LyftTarget>RightLyft.getCurrentPosition()){
                if(Math.abs(LyftTarget-RightLyft.getCurrentPosition())>50){
                   RightLyft.setPower(1);
                }else{
                   RightLyft.setPower(0);
                }


            }else if (LyftTarget<RightLyft.getCurrentPosition()){
                if(Math.abs(LyftTarget-RightLyft.getCurrentPosition())>50){

                    if(Math.abs(LyftTarget-RightLyft.getCurrentPosition())>1000){

                        RightLyft.setPower(-.7);
                    } else{
                        RightLyft.setPower(-.3);
                    }



                }else{
                    RightLyft.setPower(0);
                }

            }else{
                RightLyft.setPower(0);
            }

        }else{
            LeftLyft.setPower(0);
            RightLyft.setPower(0);

        }




    }
    public void intakeMotors(){
        if(gamepad1.y){
            LeftSucker.setPower(-.4);
            RightSucker.setPower(.4);
        }else if(gamepad1.x){
            LeftSucker.setPower(.4);
            RightSucker.setPower(-.4);

        } else{
            LeftSucker.setPower(0);
            RightSucker.setPower(0);
        }
    }
    public void extendGrabber(){
        if(gamepad2.right_bumper){
            Extender.setPosition(.75);
        }else if(gamepad2.left_bumper){
            Extender.setPosition(.25);
        }else{
            Extender.setPosition(.5);
        }
    }
    public void grabBlock(){
        if(gamepad2.right_trigger>.5){
            LeftGrabber.setPosition(.46);
            RightGrabber.setPosition(.51);
        }
        if(gamepad2.left_trigger>.5){
            LeftGrabber.setPosition(.38);
            RightGrabber.setPosition(.58);
        }
        if(gamepad2.y){
            LeftGrabber.setPosition(.34);
            RightGrabber.setPosition(.60);
        }
    }
    public void grabFoundation(){
        if(gamepad1.right_trigger>.5){

            FoundationGrabber.setPosition(.75);
        }
        if(gamepad1.left_trigger>.5){
            FoundationGrabber.setPosition(.3);
        }

    }

}
