package org.firstinspires.ftc.teamcode;


import android.media.MediaPlayer;
import android.net.rtp.AudioStream;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.io.File;

@Disabled
@TeleOp(name="KickOffRobot", group="PinktotheFuture")
public class KickOffRobot extends LinearOpMode implements org.firstinspires.ftc.teamcode.ServoVariables {
    bno055driver imu;

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;

        imu = new bno055driver("IMU", hardwareMap);

        double geleiderPw = 0;

        double fastency = 1;

        double GlyphgrabLPos = 0.5;
        double GlyphgrabRPos = 0.5;

        boolean correcting = false;

        Double[] imuArray;
        imuArray = new Double[1];
        imuArray[0] = 0.0;

        Servo RelicSlideOpenerR = hardwareMap.servo.get("relicslideopenerr");
        Servo RelicSlideOpenerL = hardwareMap.servo.get("relicslideopenerl");

        Servo GlyphgrabL = hardwareMap.servo.get("glyphgrabl");
        Servo GlyphgrabR = hardwareMap.servo.get("glyphgrabr");

        RelicSlideOpenerR.setPosition(RelicSlideRServoMIN);
        RelicSlideOpenerL.setPosition(RelicSlideLServoMAX);

        DcMotor Geleider = hardwareMap.dcMotor.get("geleider");
        Geleider.setDirection(DcMotorSimple.Direction.REVERSE);


        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");

        //RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_right) fastency =.6;
            if (gamepad1.dpad_down)   fastency = 0.4;

            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;

            geleiderPw = 0;


            RFpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);
            RBpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
            LFpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
            LBpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);


            if (gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_x < 0.1) {
                RFpower = -gamepad1.left_stick_y;
                RBpower = -gamepad1.left_stick_y;
                LFpower = -gamepad1.left_stick_y;
                LBpower = -gamepad1.left_stick_y;
            }
            if (gamepad1.left_stick_y > -0.1 && gamepad1.left_stick_y < 0.1) {
                RFpower = -gamepad1.left_stick_x;
                RBpower = gamepad1.left_stick_x;
                LFpower = gamepad1.left_stick_x;
                LBpower = -gamepad1.left_stick_x;
            }

            RFpower = RFpower - (gamepad1.right_stick_x);
            RBpower = RBpower - (gamepad1.right_stick_x);
            LFpower = LFpower + (gamepad1.right_stick_x);
            LBpower = LBpower + (gamepad1.right_stick_x);

            if (gamepad2.a) {
                GlyphgrabLPos += 0.01;
                GlyphgrabRPos -= 0.01;

            }
            if (gamepad2.b) {
                GlyphgrabLPos -= 0.01;
                GlyphgrabRPos += 0.01;
            }

            if (gamepad2.x){
                RelicSlideOpenerR.setPosition(RelicSlideRServoMAX);
                RelicSlideOpenerL.setPosition(RelicSlideLServoMIN);
            }
            if (gamepad2.y){
                RelicSlideOpenerR.setPosition(RelicSlideRServoMIN);
                RelicSlideOpenerL.setPosition(RelicSlideLServoMAX);
            }

            double temp;

            double theta = imu.getAngles()[0];

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rcw = gamepad1.right_stick_x;

            if (theta >0) {
                temp = forward*Math.cos(theta)-strafe*Math.sin(theta);
                strafe = forward*Math.sin(theta)+strafe*Math.cos(theta);
                forward = temp;
            }

            if (theta <=0) {
                temp = forward*Math.cos(theta)-strafe*Math.sin(theta);
                strafe = forward*Math.sin(theta)+strafe*Math.cos(theta);
                forward = temp;
            }

            if (Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0 || Math.abs(gamepad1.right_stick_y) > 0 ){
                imuArray[0] = theta;
            }

            double oldAngle = imuArray[0]*180/Math.PI;
            double newAngle = theta*180/Math.PI;

            double rawDiff = oldAngle > newAngle ? oldAngle - newAngle : newAngle - oldAngle;

            if (theta < 0){
                rawDiff = rawDiff * -1;
            }





            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;


            LFpower = forward+rcw+strafe;
            RFpower = forward-rcw-strafe;
            LBpower = forward+rcw-strafe;
            RBpower = forward-rcw+strafe;

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);


            GlyphgrabLPos = Range.clip(GlyphgrabLPos,0.4,1);
            GlyphgrabRPos = Range.clip(GlyphgrabRPos,0,0.6);
            GlyphgrabL.setPosition(GlyphgrabLPos);
            GlyphgrabR.setPosition(GlyphgrabRPos);

            if (Math.abs(gamepad1.left_stick_x) == 0 && Math.abs(gamepad1.left_stick_y) == 0 && Math.abs(gamepad1.right_stick_x) ==  0 && Math.abs(gamepad1.right_stick_y) == 0){
                if (rawDiff > 5.0){
                    LFpower = 0.2;
                    LBpower = 0.2;
                    RFpower = -0.2;
                    RBpower = -0.2;
                }

                if (rawDiff < -5.0){
                    LFpower = -0.2;
                    LBpower = -0.2;
                    RFpower = 0.2;
                    RBpower = 0.2;
                }

                correcting = true;
            }else{
                correcting = false;
            }

            LFdrive.setPower(LFpower * fastency);
            RBdrive.setPower(RBpower * fastency);
            LBdrive.setPower(LBpower * fastency);
            RFdrive.setPower(RFpower * fastency);

            Geleider.setPower(gamepad2.right_stick_y);

            //telemetry.addData("LB",LBpower);
            //telemetry.addData("LF",LFpower);
            //telemetry.addData("RB",RBpower);
            //telemetry.addData("RF",RFpower);
            telemetry.addData("array", imuArray[0]);
            telemetry.addData("raw", theta);
            telemetry.addData("rawDiff", rawDiff);
            telemetry.addData("correcting is:", correcting);
            telemetry.update();


        }
    }
}
