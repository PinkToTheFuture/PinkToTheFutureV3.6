package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Vector;


/**
 * Created by seb7 on 15-3-2017.
 */

//Autonomous beta mecanum BLUE with vuforia
@Autonomous (name="Vuforia beacon push", group="PinktotheFuture")
@Disabled
public class VuforiaBeaconPush extends LinearOpMode {



    public void Forward(double omw, double pwr) throws InterruptedException{
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");


        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);


        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //omwentelingen invoeren, encoder  ticks uitkrijgen
        double ticks = omw * 1478.4;


        LFdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (LBdrive.getCurrentPosition() < ticks || RBdrive.getCurrentPosition() < ticks && opModeIsActive()){
            waitOneFullHardwareCycle();
            if (LBdrive.getCurrentPosition() > ticks){
                LFdrive.setPower(0);
                LBdrive.setPower(0);
            }
            if (RBdrive.getCurrentPosition() > ticks) {
                RFdrive.setPower(0);
                RBdrive.setPower(0);
            }
            if (RBdrive.getCurrentPosition() < LBdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.7);
                LBdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 1.3);
            }
            if (RBdrive.getCurrentPosition() > LBdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.3);
                LBdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 0.7);
            }
            telemetry.addData("L", LBdrive.getCurrentPosition());
            telemetry.addData("R", RBdrive.getCurrentPosition());
            telemetry.update();
        }
        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);
    }
    public void Drive(double R, double L, double pwr) throws InterruptedException{  //een void voor al het driven
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");


        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);


        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //omwentelingen invoeren, encoder  ticks uitkrijgen
        double Rticks = R * 1478.4;
        double Lticks = L * 1478.4;


        if (!(L == 0)){
            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
        }
        if (!(R==0)){
            RBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
        }

        while (LFdrive.getCurrentPosition() < Lticks || RFdrive.getCurrentPosition() < Rticks && opModeIsActive()){
            waitOneFullHardwareCycle();
            if (RFdrive.getCurrentPosition() > LFdrive.getCurrentPosition() && R == L){
                LFdrive.setPower(pwr * 0.7);
                LBdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 1.3);
            }
            if (RFdrive.getCurrentPosition() < LFdrive.getCurrentPosition() && R == L){
                LFdrive.setPower(pwr * 1.3);
                LBdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 0.7);
            }
            if (LFdrive.getCurrentPosition() > Lticks){
                LFdrive.setPower(0);
                LBdrive.setPower(0);
            }
            if (RFdrive.getCurrentPosition() > Rticks) {
                RFdrive.setPower(0);
                RBdrive.setPower(0);
            }
            telemetry.addData("L", LFdrive.getCurrentPosition());
            telemetry.addData("R", RFdrive.getCurrentPosition());
            telemetry.update();
        }
        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);

    }
    public void RechtZetten(double h, double pwr) throws InterruptedException {

        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");

        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LightSensor Rlight = hardwareMap.lightSensor.get("Rlight");
        LightSensor Llight = hardwareMap.lightSensor.get("Llight");

        Rlight.enableLed(true);
        Llight.enableLed(true);

        LFdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (Rlight.getLightDetected() < h || Llight.getLightDetected() < h  && opModeIsActive()) {
            waitOneFullHardwareCycle();
            if (Rlight.getLightDetected() > h){
                RFdrive.setPower(0);
                RBdrive.setPower(0);
            }
            if (Llight.getLightDetected() > h){
                LFdrive.setPower(0);
                LBdrive.setPower(0);
            }
        }
        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);
        Rlight.enableLed(false);
        Llight.enableLed(false);
    }
    public void Reverse(double omw, double pwr) throws InterruptedException{
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");


        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD); //alles andersom omdat we achteruit gaan
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);


        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //omwentelingen invoeren, encoder  ticks uitkrijgen
        double ticks = omw * 1000;


        LFdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (LBdrive.getCurrentPosition() < ticks || RBdrive.getCurrentPosition() < ticks && opModeIsActive()){
            waitOneFullHardwareCycle();
            if (LBdrive.getCurrentPosition() > ticks){
                LFdrive.setPower(0);
                LBdrive.setPower(0);
            }
            if (RBdrive.getCurrentPosition() > ticks) {
                RFdrive.setPower(0);
                RBdrive.setPower(0);
            }
            if (RBdrive.getCurrentPosition() < LBdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.7);
                LBdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 1.3);
            }
            if (RBdrive.getCurrentPosition() > LBdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.3);
                LBdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 0.7);
            }
            telemetry.addData("L", LBdrive.getCurrentPosition());
            telemetry.addData("R", RBdrive.getCurrentPosition());
            telemetry.update();
        }
        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);
    }
    public void Select(String color) throws InterruptedException{
        Servo lbs = hardwareMap.servo.get("lbs");
        Servo rbs = hardwareMap.servo.get("rbs");

        ColorSensor Rcolor = hardwareMap.colorSensor.get("Rcolor");
        ColorSensor Lcolor = hardwareMap.colorSensor.get("Lcolor");
        Lcolor.setI2cAddress(I2cAddr.create8bit(0x3c));
        Rcolor.setI2cAddress(I2cAddr.create8bit(0x2c));


        float LhsvValues[] = {0F,0F,0F};
        float RhsvValues[] = {0F,0F,0F};


        boolean loop = true;
        while (loop && opModeIsActive()) {
            waitOneFullHardwareCycle();
            Color.RGBToHSV(Lcolor.red(), Lcolor.green(), Lcolor.blue(), LhsvValues);
            Color.RGBToHSV(Rcolor.red(), Rcolor.green(), Rcolor.blue(), RhsvValues);

            double Lblue = Math.floor(Lcolor.blue());
            double Lred = Math.floor(Lcolor.red());
            double Rblue = Math.floor(Rcolor.blue());
            double Rred = Math.floor(Rcolor.red());
            telemetry.addData("", Lcolor.red());

            if (color == "red") {
                telemetry.addData("","red");
                if (Lblue > Lred) {
                    telemetry.addData("Lblue > Lred", "");
                    Reverse(0.3, 0.3);
                    lbs.setPosition(0);
                    sleep(500);
                    lbs.setPosition(1);
                    sleep(200);
                    lbs.setPosition(0);
                    sleep(500);
                    lbs.setPosition(1);
                    loop = false;
                }
                if (Lred > Lblue) {
                    telemetry.addData("Lred > Lblue", "");
                    lbs.setPosition(0);
                    sleep(500);
                    lbs.setPosition(0.6);
                    sleep(200);
                    lbs.setPosition(0);
                    sleep(500);
                    lbs.setPosition(0.6);
                    loop = false;
                }
            }
            if (color == "blue"){
                telemetry.addData("", "blue");
                if (Rblue > Rred) {
                    telemetry.addData("Rblue > Rred", "");
                    rbs.setPosition(0.6);
                    sleep(500);
                    rbs.setPosition(0);
                    sleep(200);
                    rbs.setPosition(0.6);
                    sleep(500);
                    rbs.setPosition(0);
                    loop = false;
                }
                if (Rred > Rblue) {
                    telemetry.addData("Rred > Rblue", "");
                    Reverse(0.3, 0.3);
                    rbs.setPosition(0.6);
                    sleep(500);
                    rbs.setPosition(0);
                    sleep(200);
                    rbs.setPosition(0.6);
                    sleep(500);
                    rbs.setPosition(0);
                    loop = false;
                }
            }
            telemetry.update();

        }
    }



    public void shoot(double omw, double pwr) throws InterruptedException{
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //omwentelingen invoeren, encoder ticks uitkrijgen
        double  ticks = omw * 1440;

        shooter.setPower(-pwr);

        while (shooter.getCurrentPosition() < ticks && opModeIsActive()){
            waitOneFullHardwareCycle();
            telemetry.addData("shooter", shooter.getCurrentPosition());
            telemetry.update();
        }
        shooter.setPower(0);
    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }


    @Override
    public  void runOpMode() throws InterruptedException {

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");

        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);


        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo lbs = hardwareMap.servo.get("lbs");
        Servo rbs = hardwareMap.servo.get("rbs");
        Servo shooterservo = hardwareMap.servo.get("shooterservo");
        shooterservo.setPosition(1);
        lbs.setPosition(1);
        rbs.setPosition(0);


        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AWQ0Ulb/////AAAAGUV8ySxRVUpulp7a+YCjrmlZm2JeXuurkO07bfKUBkHpzLT+YSpn4CoCa0npzqKJIpaEN/8tZNkb7O2n+9SVsHg8J3Ym3QOfQecp6DZ7Tui8+Xmn7LS8nHhTPF7Uu3ghmXYh3GGw8PRgBH/+Imr+zIeiIzMr7zB86pPpXcgJ+6tsGG1gJSfbnQrNkLXAmFNjD5ukobtg6JQs6yaOvHUcRXNCtV1XZVswFY130QGadPmiwiHNKABotOkGuxEb1ql2e8pwRBgjTh35o7tPH0OUGtsMawk/y2Ad2K0hkzVQKZDLmwAnAfb012tcR+8aXTHDQXEcZJ73z3yYWOFl4Jdvox4FooGttkJ30LORvX9gxJFS";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 2);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Legos");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
        VuforiaTrackableDefaultListener Legos = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();

        waitForStart();

        Forward(0.5, 0.25);
        sleep(100);
        shoot(1.5, 0.5);
        shooterservo.setPosition(0.5);
        sleep(100);
        shoot(1.5, 0.5);
        sleep(100);

        //rijden tot dat hij de beacon ziet
        LFdrive.setPower(0.2);
        RFdrive.setPower(-0.2);
        LBdrive.setPower(-0.2);
        RBdrive.setPower(0.2);
        while (opModeIsActive() && wheels.getRawPose() == null)
        { waitOneFullHardwareCycle();
            idle();
        }
        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);
        sleep(100);


        //naar beacon rijden
        VectorF angles = anglesFromTarget(wheels);
        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

        if (trans.get(0) > 0) {
            //draai links
            LFdrive.setPower(-0.2);
            RFdrive.setPower(0.2);
            LBdrive.setPower(-0.2);
            RBdrive.setPower(0.2);

        } else {
            //draai rechts
            LFdrive.setPower(0.2);
            RFdrive.setPower(-0.2);
            LBdrive.setPower(0.2);
            RBdrive.setPower(-0.2);
        }

        do {
            if (wheels.getPose() != null) {
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0))>30);

        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);


        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //positie bereken wielen
        //LFdrive.setTargetPosition((int)(LFdrive.getCurrentPosition()+(Math.hypot(trans.get(0), trans.get(2)+230)/100*1478.4)));
        //RFdrive.setTargetPosition((int)(RFdrive.getCurrentPosition()+(Math.hypot(trans.get(0), trans.get(2)+230)/100*1478.4)));
        //LBdrive.setTargetPosition((int)(LBdrive.getCurrentPosition()+(Math.hypot(trans.get(0), trans.get(2)+230)/100*1478.4)));
        //RBdrive.setTargetPosition((int)(RBdrive.getCurrentPosition()+(Math.hypot(trans.get(0), trans.get(2)+230)/100*1478.4)));

        while (opModeIsActive() && LFdrive.isBusy() && RFdrive.isBusy() && LBdrive.isBusy() && RBdrive.isBusy()) {
            waitOneFullHardwareCycle();
            idle();
        }

        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);

        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10) {
            waitOneFullHardwareCycle();
            if (wheels.getPose() != null) {
                if (Math.abs(wheels.getPose().getTranslation().get(0)) > 0) {
                    LFdrive.setPower(-0.2);
                    RFdrive.setPower(0.2);
                    LBdrive.setPower(-0.2);
                    RBdrive.setPower(0.2);
                } else {
                    LFdrive.setPower(0.2);
                    RFdrive.setPower(-0.2);
                    LBdrive.setPower(0.2);
                    RBdrive.setPower(-0.2);
                }
            } else {
                LFdrive.setPower(-0.2);
                RFdrive.setPower(0.2);
                LBdrive.setPower(-0.2);
                RBdrive.setPower(0.2);

                }

            LFdrive.setPower(0);
            RFdrive.setPower(0);
            LBdrive.setPower(0);
            RBdrive.setPower(0);
            sleep(100);
            }


        }

    }


