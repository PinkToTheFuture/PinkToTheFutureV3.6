package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@Autonomous(name = "10 sec shoot corner blue", group = "corner")

public class TenSecShootCornerBlue extends LinearOpMode implements org.firstinspires.ftc.teamcode.ServoVariables {
    public void Forward(double omw, double pwr) throws InterruptedException{
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        LFdrive.setTargetPosition((int) (omw * 11.20));
        LBdrive.setTargetPosition((int) (omw * 11.20));
        RFdrive.setTargetPosition((int) (omw * 11.20));
        RBdrive.setTargetPosition((int) (omw * 11.20));



        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.50);
                LBdrive.setPower(pwr * 1.50);
                RFdrive.setPower(pwr * 0.66);
                RBdrive.setPower(pwr * 0.66);
            }
            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.66);
                LBdrive.setPower(pwr * 0.66);
                RFdrive.setPower(pwr * 1.50);
                RBdrive.setPower(pwr * 1.50);
            }
            if (LFdrive.getCurrentPosition() > ((omw * 11.20) - 40) && LBdrive.getCurrentPosition() > ((omw * 11.20) - 40) && RFdrive.getCurrentPosition() > ((omw * 11.20) - 40) && RBdrive.getCurrentPosition() > ((omw * 11.20) - 40)){
                loop = false;
                telemetry.addData("loop is false", "");
                telemetry.update();
                sleep(500);
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

    }

    public void shoot() throws InterruptedException{
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");

        TouchSensor shootertouch = hardwareMap.touchSensor.get("shootertouch");


        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(1);
        shooter.setTargetPosition(2240);

        boolean loop = true;
        while (loop && opModeIsActive()){
            if (shooter.getCurrentPosition() > 2220){
                loop = false;
            }
        }

        shooterservoX.setPosition(0.15);
        sleep(200);
        while (opModeIsActive() && !shootertouch.isPressed()){
            idle();
        }
        sleep(140);
        shooterservoX.setPosition(0.5);


        sleep(300);
        shooter.setTargetPosition(4480);

        loop = true;
        while (loop && opModeIsActive()){
            if (shooter.getCurrentPosition() > 4460){
                loop = false;
            }
        }
    }


    private void Right_Gyro(double degrees, double pwr, double sloommultiplier) throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && gyro.getHeading() < (degrees - 10) || gyro.getHeading() > 340){
            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(-pwr);
            RBdrive.setPower(-pwr);
            idle();
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.update();

        }
        boolean loop = true;
        while (opModeIsActive() && loop){
            while (opModeIsActive() && !(gyro.getHeading() == degrees)){
                if (gyro.getHeading() > degrees) {
                    LFdrive.setPower(-pwr * sloommultiplier);
                    LBdrive.setPower(-pwr * sloommultiplier);
                    RFdrive.setPower(pwr * sloommultiplier);
                    RBdrive.setPower(pwr * sloommultiplier);
                }
                if (gyro.getHeading() < degrees) {
                    LFdrive.setPower(pwr * sloommultiplier);
                    LBdrive.setPower(pwr * sloommultiplier);
                    RFdrive.setPower(-pwr * sloommultiplier);
                    RBdrive.setPower(-pwr * sloommultiplier);
                }
                telemetry.addData("gyro", gyro.getHeading());
                telemetry.update();
            }
            LFdrive.setPower(0);
            LBdrive.setPower(0);
            RFdrive.setPower(0);
            RBdrive.setPower(0);
            sleep(50);
            if (gyro.getHeading() == degrees){
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }
    private void init_gyro() throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");
        gyro.calibrate();
        while (gyro.isCalibrating()){
            idle();
        }
        telemetry.addData("gyro calibrated", gyro.status());
        telemetry.update();
    }

    private void Left_Sideways(double omw, double pwr) throws InterruptedException {
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        LFdrive.setTargetPosition((int) (omw * 11.20));
        LBdrive.setTargetPosition((int) (omw * 11.20));
        RFdrive.setTargetPosition((int) (omw * 11.20));
        RBdrive.setTargetPosition((int) (omw * 11.20));

        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.75);
            }
            if (LFdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.33);
            }


            if (LBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr * 0.75);
            }
            if (LBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr * 1.33);
            }


            if (RBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr * 0.75);
            }
            if (RBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr * 1.33);
            }


            if (LFdrive.getCurrentPosition() > (omw*11.20 - 40) && LBdrive.getCurrentPosition() > (omw*11.20 - 40) && RFdrive.getCurrentPosition() > (omw*11.20 - 40) && RBdrive.getCurrentPosition() > (omw*11.20 - 40)) {
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    @Override public void runOpMode() throws InterruptedException {
        Servo Armservo = hardwareMap.servo.get("servoarm");
        Armservo.setPosition(ArmservoStopPosition);

        Servo releaseArmL = hardwareMap.servo.get("releasearmL");
        Servo releaseArmR = hardwareMap.servo.get("releasearmR");
        releaseArmL.setPosition(releaseArmLStartPosition);
        releaseArmR.setPosition(releaseArmRStartPosition);
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");
        shooterservoX.setPosition(shooterservoXStartPosition);

        init_gyro();
        waitForStart();
        sleep(5000);
        Forward(125, 0.4);
        Left_Sideways(260, 0.4);
        shoot();
        Right_Gyro(100, 0.29, 0.48);
        Forward(300, 0.4);

    }
}
