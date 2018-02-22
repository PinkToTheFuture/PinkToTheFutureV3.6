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


@TeleOp(name="PWS AVOND", group="PinktotheFuture")
public class PWSavond extends LinearOpMode {
    bno055driver imu2;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;

        double jewelpos =0;

        boolean correcting = false;

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");

        Servo Jewelservo = hardwareMap.servo.get("Jewelservo");

        //RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);


        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        double speed = 1;

        //double K1 = 0.5; //increase value not higher than 1

        //rcw = rcw*K1;

        Double[] imuArray;
        imuArray = new Double[1];
        imuArray[0] = 0.0;

        imu2 = new bno055driver("IMU", hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "IMU");


        LFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (opModeIsActive()) {

            RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
            RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
            LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
            LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

            speed=1;
            if (gamepad1.dpad_up)       speed = speed+.2;
            if (gamepad1.dpad_right)    speed =1;
            if (gamepad1.dpad_down)     speed = speed-.2;

            if (gamepad2.a) {
                jewelpos =1;
            }
            if (gamepad2.b) {
                jewelpos =0;
            }

            double temp;


            double theta = imu2.getAngles()[0];

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rcw = gamepad1.right_stick_x;

            if (Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0 || Math.abs(gamepad1.right_stick_y) > 0 ){
                imuArray[0] = theta;
            }

            double oldAngle = imuArray[0]*180/Math.PI;
            double newAngle = theta*180/Math.PI;

            double rawDiff = oldAngle > newAngle ? oldAngle - newAngle : newAngle - oldAngle;


            if (theta < 0){
                rawDiff = rawDiff * -1;
            }

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

            if (gamepad1.y) {
                RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);

                //The PID controlller:

                //double Pitchv = 1;     //pitch value (change to 1 to test PID)
                //double Rollv = 1;     //roll value (change to 1 to test PID)


                double Kp1 = 2;
                double Ki1 = 2;
                double Kd1 = 2;

                double Kp2 = 2;
                double Ki2 = 2;
                double Kd2 = 2;

                double error1;
                double error2;

                double Sp = 0;
                double Pv1 = imu2.getAngles()[1];
                double Pv2 = imu2.getAngles()[2];

                double integral1 = 0;
                double integral2 = 0;
                double derivative1;
                double derivative2;


                error1 = Sp - Pv1;
                error2 = Sp - Pv2;

                integral1 = error1 + integral1;
                integral2 = error2 + integral2;

                double lasterror1 = error1;
                double lasterror2 = error2;

                derivative1 = error1 - lasterror1;
                derivative2 = error2 - lasterror2;

                double correction1; //value for the motors from the pitch value
                double correction2; //value for the motors from the roll value

                correction1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
                correction2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;

                //End of PID controller

                RFpower = ((((correction1) + (-correction2)) / 2));
                RBpower = ((((correction1) - (-correction2)) / 2));
                LFpower = ((((correction1) - (-correction2)) / 2));
                LBpower = ((((correction1) + (-correction2)) / 2));

                //RIGHT STICK

                Range.clip(RFpower, -1, 1);
                Range.clip(RBpower, -1, 1);
                Range.clip(LFpower, -1, 1);
                Range.clip(LBpower, -1, 1);

                LFdrive.setPower(LFpower);
                RFdrive.setPower(RFpower);
                LBdrive.setPower(LBpower);
                RBdrive.setPower(RBpower);
            }

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);


            LFdrive.setPower(LFpower * speed);
            RBdrive.setPower(RBpower * speed);
            LBdrive.setPower(LBpower * speed);
            RFdrive.setPower(RFpower * speed);

            Jewelservo.setPosition(jewelpos);

            telemetry.addData("angle: ", theta);
            telemetry.addData("rawDiff", rawDiff);
            telemetry.addData("correcting is:", correcting);
            telemetry.update();

        }
    }


}
