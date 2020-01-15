package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous


    public class RedSkyStone extends LinearOpMode {

    DcMotor FL, FR, BL, BR;
    Servo josh, daisy;
    private final double JOSH_UP = .12;
    private final double JOSH_DOWN = .5;
    private final double DAISY_UP = .86;
    private final double DAISY_DOWN = .5;

    Servo Sayra;
    private final double SAYRA_UP = .54;
    private final double SAYRA_DOWN = .17;

    private DistanceSensor Sayra2;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    ColorSensor tapesensor;
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public void runOpMode() {

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        josh = hardwareMap.servo.get("josh");
        josh.setPosition(JOSH_UP);
        daisy = hardwareMap.servo.get("daisy");
        daisy.setPosition(DAISY_UP);

        Sayra = hardwareMap.servo.get("Sayra");
        Sayra.setPosition(SAYRA_UP);

        Sayra2 = hardwareMap.get(DistanceSensor.class, "Sayra2");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        tapesensor = hardwareMap.get(ColorSensor.class, "tapesensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        composeTelemetry();


        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        //test the hue of the red and blue
        /*while(opModeIsActive()){
            Color.RGBToHSV((int) (tapesensor.red() * SCALE_FACTOR),
                    (int) (tapesensor.green() * SCALE_FACTOR),
                    (int) (tapesensor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Red",tapesensor.red());19192
            telemetry.addData("Blue",tapesensor.blue());
            telemetry.update();
        }*/

        drive(.25,.25);
        sleep(2500);
        drive(0,0);
        sleep(200);

        strafeLeft(false,.3);
        while(Sayra2.getDistance(DistanceUnit.CM)>8){
            sleep(20);
        }
        drive(0,0);
        sleep(500);

        drive(-.3,-.3);
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        while( hsvValues[0] <100 ){

            sleep(20);
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

        }
        drive(0,0);
        sleep(500);

        grabSkyStone(true);
        sleep(200);
        drive(0,0);
        sleep(200);

        strafeLeft(true,.3);
        sleep(2800);
        drive(0,0);
        sleep(200);

        driveToTape(0,-.4);
        sleep(500);
        drive(0,0);
        sleep(200);

        drive(-.3,-.3);
        sleep(400);
        drive(0,0);
        sleep(200);

        grabSkyStone(false);
        sleep(200);
        drive(0,0);
        sleep(500);

        driveToTape(0,.3);
        sleep(500);
        drive(0,0);








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

    public void lockFoundation(boolean lock) {

        if (lock) {

            daisy.setPosition(DAISY_DOWN);
            josh.setPosition(JOSH_DOWN);

        } else {

            daisy.setPosition(DAISY_UP);
            josh.setPosition(JOSH_UP);

        }

    }

    public void gyroTurnLeft(boolean left, double power, double degrees) {
        telemetry.addData("current heading", angles.firstAngle);
        if (left) {

            FR.setPower(power);
            FL.setPower(-power);
            BL.setPower(-power);
            BR.setPower(power);

            //if turn to left, positive heading
            while (angles.firstAngle < degrees) {
                sleep(20);
                telemetry.update();

            }


        } else {

            FR.setPower(-power);
            FL.setPower(power);
            BL.setPower(power);
            BR.setPower(-power);
            degrees = -degrees;
            while (angles.firstAngle > degrees) {
                sleep(20);
                telemetry.update();

            }


        }

        FR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void turnToHeading(double degrees) {
        if (angles.firstAngle > degrees) {
            drive(.2, -.2);
            while (angles.firstAngle > degrees) {
                sleep(10);
                telemetry.update();

            }
        } else if (angles.firstAngle < degrees) {
            drive(-.2, .2);
            while (angles.firstAngle < degrees) {
                sleep(10);
                telemetry.update();
            }
        } else {
            drive(0, 0);
        }

        drive(0, 0);
        sleep(500);


    }

    public void driveWithHeading(double degrees, double time) {
        double endtime = getRuntime() + time;
        while (getRuntime() < endtime) {
            telemetry.addData("current get run time", getRuntime());
            telemetry.addData("value of end time", endtime);
            telemetry.update();
            if (angles.firstAngle - degrees < -2) {
                drive(-.3, -.2);

            } else if (angles.firstAngle - degrees > 2) {
                drive(-.2, -.3);
            } else {

                drive(-.3, -.3);


            }

        }
    }

    public void grabSkyStone(boolean grab) {

        if (grab) {
            Sayra.setPosition(SAYRA_DOWN);
        } else {
            Sayra.setPosition(SAYRA_UP);
        }
    }

    public void driveToTape(double degrees, double power) {

        if (power > 0) {
            drive(power, power);
            while (tapesensor.red() < 100 && tapesensor.blue() < 100) {
                sleep(20);

                telemetry.addData("current heading", angles.firstAngle);
                telemetry.update();
                if (angles.firstAngle - degrees < -2) {
                    drive(power, power + .1);

                } else if (angles.firstAngle - degrees > 2) {
                    drive(power + .1, power);
                } else {

                    drive(power, power);

                }


            }
        } else {

            drive(power, power);
            while (tapesensor.red() < 100 && tapesensor.blue() < 100) {
                sleep(20);

                telemetry.addData("current heading", angles.firstAngle);
                telemetry.update();
                if (angles.firstAngle - degrees < -2) {
                    drive(power-.1, power);

                } else if (angles.firstAngle - degrees > 2) {
                    drive(power , power-.1);
                } else {

                    drive(power, power);


                }



            }


        }
        drive(0, 0);
    }
}