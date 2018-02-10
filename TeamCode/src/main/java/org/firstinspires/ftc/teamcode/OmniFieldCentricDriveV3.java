package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="OmniFieldCentricDriveV3", group="PinktotheFuture")
public class OmniFieldCentricDriveV3 extends LinearOpMode {
    bno055driver imu2;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;


        double speed = 1;

        //double K1 = 0.5; //increase value not higher than 1

        //rcw = rcw*K1;


        imu2 = new bno055driver("IMU", hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "IMU");

        Double[] imuArray;
        imuArray = new Double[1];



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
            if (gamepad1.dpad_up)     speed = 1;
            if (gamepad1.dpad_right) speed =.6;
            if (gamepad1.dpad_down)   speed = 0.4;

            double temp;


            double theta = imu2.getAngles()[2];

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

            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;


            LFpower = forward+rcw+strafe;
            RFpower = forward-rcw-strafe;
            LBpower = forward+rcw-strafe;
            RBpower = forward-rcw+strafe;


            if (gamepad1.b) {
                LFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                LBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (gamepad1.a) {
                LFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                RBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                LBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                RFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }


            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);


            LFdrive.setPower(LFpower * speed);
            RBdrive.setPower(RBpower * speed);
            LBdrive.setPower(LBpower * speed);
            RFdrive.setPower(RFpower * speed);

            /*telemetry.addData("Yaw(rad): ", theta);
            telemetry.addData("Yaw(deg): ", theta*180/Math.PI);
            telemetry.addData("temp", forward);
            telemetry.addData("strafe: ", strafe);

            telemetry.addData("LB",Math.round(LBpower));
            telemetry.addData("LF",Math.round(LFpower));
            telemetry.addData("RB",Math.round(RBpower));
            telemetry.addData("RF",Math.round(RFpower));
            */
            telemetry.addData("acc: ", imu.getAcceleration());
            telemetry.addData("accLin: ", imu.getLinearAcceleration());
            telemetry.addData("accOveral: ", imu.getOverallAcceleration());

            telemetry.addData("angle: ", theta);
            telemetry.update();


        }
    }
}
