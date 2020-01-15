package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class EncoderTest extends OpMode {

    DcMotor LeftLyft;

    public void init(){

       LeftLyft = hardwareMap.dcMotor.get("LeftLyft");
        LeftLyft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       LeftLyft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void loop(){

        if(gamepad1.left_bumper){
            LeftLyft.setPower(.2);
        }else if(gamepad1.right_bumper){
            LeftLyft.setPower(-.2);
        }else{
            LeftLyft.setPower(0);
        }

        telemetry.addData("Encoder Value",LeftLyft.getCurrentPosition());
        telemetry.update();
    }
}
