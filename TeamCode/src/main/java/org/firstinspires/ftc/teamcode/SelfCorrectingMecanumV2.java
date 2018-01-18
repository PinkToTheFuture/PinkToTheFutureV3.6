package org.firstinspires.ftc.teamcode;



import android.app.ActivityManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Convert;



@TeleOp(name="SelfCorrectingMecanumV2", group="PinktotheFuture")
public class SelfCorrectingMecanumV2 extends LinearOpMode {
    bno055driver imu2;
    BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double fastency = 1;

        boolean correcting = false;

        Double[] encoderArrayLF;
        Double[] encoderArrayRB;
        Double[] encoderArrayLB;
        Double[] encoderArrayRF;

        encoderArrayLF = new Double[1];
        encoderArrayRB = new Double[1];
        encoderArrayLB = new Double[1];
        encoderArrayRF = new Double[1];

        encoderArrayLF[0] = 0.0;
        encoderArrayLB[0] = 0.0;
        encoderArrayRF[0] = 0.0;
        encoderArrayRB[0] = 0.0;

        imu2 = new bno055driver("IMU", hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "IMU");

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");

        //RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_right) fastency =.5;
            if (gamepad1.dpad_down)   fastency = 0.3;

            double temp;

            double theta = imu2.getAngles()[0];

            double LBpos = LBdrive.getCurrentPosition();
            double LFpos = LFdrive.getCurrentPosition();
            double RBpos = RBdrive.getCurrentPosition();
            double RFpos = RFdrive.getCurrentPosition();


            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rcw = gamepad1.right_stick_x;


            if (Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0 || Math.abs(gamepad1.right_stick_y) > 0 ){
                encoderArrayLB[0] = LBpos;
                encoderArrayLF[0] = LFpos;
                encoderArrayRB[0] = RBpos;
                encoderArrayRF[0] = RFpos;


            }

            double oldPositionLF = LFpos - encoderArrayLF[0];
            double oldPositionLB = LBpos - encoderArrayLB[0];
            double oldPositionRF = RFpos - encoderArrayRF[0];
            double oldPositionRB = RBpos - encoderArrayRB[0];


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

            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;

            LFpower = forward+rcw+strafe;
            RFpower = forward-rcw-strafe;
            LBpower = forward+rcw-strafe;
            RBpower = forward-rcw+strafe;

            if (Math.abs(gamepad1.left_stick_x) == 0 && Math.abs(gamepad1.left_stick_y) == 0 && Math.abs(gamepad1.right_stick_x) ==  0 && Math.abs(gamepad1.right_stick_y) == 0){
                double LFpwr;
                double LBpwr;
                double RFpwr;
                double RBpwr;

                if (oldPositionLF<0) {
                    LFpwr = -.3;
                }
                else {
                    LFpwr =0.3;
                }
                if (oldPositionLB<0) {
                    LBpwr = -.3;
                }
                else {
                    LBpwr =0.3;
                }
                if (oldPositionRF<0) {
                    RFpwr = -.3;
                }
                else {
                    RFpwr =0.3;
                }
                if (oldPositionRB<0) {
                    RBpwr = -.3;
                }
                else {
                    RBpwr =0.3;
                }


                LFdrive.setTargetPosition((int) oldPositionLF);
                LBdrive.setTargetPosition((int) oldPositionLB);
                RFdrive.setTargetPosition((int) oldPositionRF);
                RBdrive.setTargetPosition((int) oldPositionRB);

                LFdrive.setPower(LFpwr);
                LBdrive.setPower(LBpwr);
                RFdrive.setPower(RFpwr);
                RBdrive.setPower(RBpwr);

                correcting = true;

            }else{

                correcting = false;
            }


            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);


            LFdrive.setPower(LFpower * fastency);
            RBdrive.setPower(RBpower * fastency);
            LBdrive.setPower(LBpower * fastency);
            RFdrive.setPower(RFpower * fastency);

            //telemetry.addData("imuArray: ", imuArray[0]);
            //telemetry.addData("imu: ", imu2.getAngles()[0]);
            /*telemetry.addData("LB",LBpower);
            telemetry.addData("LF",LFpower);
            telemetry.addData("RB",RBpower);
            telemetry.addData("RF",RFpower);
            */

            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());

            telemetry.update();


        }
    }



}
