package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
@Disabled
@Autonomous(name = "AUTO BLUE", group = "full")

public class PTTF_AUTO_BLUE extends LinearOpMode implements org.firstinspires.ftc.teamcode.ServoVariables {
    private void Forward(double omw, double pwr) throws InterruptedException{
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
        int marge = (int)((omw * 11.20) - 40);


        while (loop && opModeIsActive()){
            telemetry.addLine("forward");
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.50);
                LBdrive.setPower(pwr * 1.50);
                RFdrive.setPower(pwr * 0.66);
                RBdrive.setPower(pwr * 0.66);
            }
            if (LFdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.66);
                LBdrive.setPower(pwr * 0.66);
                RFdrive.setPower(pwr * 1.50);
                RBdrive.setPower(pwr * 1.50);
            }
            if (LFdrive.getCurrentPosition() > marge && LBdrive.getCurrentPosition() > marge && RFdrive.getCurrentPosition() > marge && RBdrive.getCurrentPosition() > marge){
                loop = false;
                telemetry.addData("loop is false", "");
                telemetry.update();
                sleep(400);
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

    }
    private void Reverse(double omw, double pwr) throws InterruptedException{
        boolean loop = true;
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
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        LFdrive.setTargetPosition((int) omw * 1120);
        LBdrive.setTargetPosition((int) omw * 1120);
        RFdrive.setTargetPosition((int) omw * 1120);
        RBdrive.setTargetPosition((int) omw * 1120);

        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.75);
                LBdrive.setPower(pwr * 0.75);
                RFdrive.setPower(pwr * 1.33);
                RBdrive.setPower(pwr * 1.33);
            }
            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.33);
                LBdrive.setPower(pwr * 1.33);
                RFdrive.setPower(pwr * 0.75);
                RBdrive.setPower(pwr * 0.75);
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

    private void Right_Gyro(double degrees, double pwr, double sloommultiplier) throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");


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

    private void Left_Gyro(double degrees, double pwr, double sloommultiplier) throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");

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
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && (gyro.getHeading() < 180 || gyro.getHeading() > (degrees + 35))){
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);
            LFdrive.setPower(-pwr);
            LBdrive.setPower(-pwr);
            idle();
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.update();

        }

        while (opModeIsActive() && !(gyro.getHeading() == (degrees))){
            if (gyro.getHeading() < degrees) {
                RFdrive.setPower(-pwr * sloommultiplier);
                RBdrive.setPower(-pwr * sloommultiplier);
                LFdrive.setPower(pwr * sloommultiplier);
                LBdrive.setPower(pwr * sloommultiplier);
            }
            if (gyro.getHeading() > degrees) {
                LFdrive.setPower(-pwr * sloommultiplier);
                LBdrive.setPower(-pwr * sloommultiplier);
                RFdrive.setPower(pwr * sloommultiplier);
                RBdrive.setPower(pwr * sloommultiplier);
            }
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.update();

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    private void init_gyro() throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");
        gyro.calibrate();
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        while (gyro.isCalibrating()){
            idle();
        }
        telemetry.addData("gyro calibrated", gyro.status());
        telemetry.update();
    }

    private void shoot() throws InterruptedException{
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

        shooterservoX.setPosition(0.13);
        sleep(400);
        while (opModeIsActive() && !shootertouch.isPressed()){
            idle();
        }
        shooterservoX.setPosition(0.5);

        shooter.setTargetPosition(4480);

        loop = true;
        while (loop && opModeIsActive()){
            if (shooter.getCurrentPosition() > 4200){
                loop = false;
            }
        }
    }

    private void Left_Sideways(double omw, double pwr) throws InterruptedException {
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

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

    private void Right_Sideways(double omw, double pwr) throws InterruptedException {
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);

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


    private void Push1() throws InterruptedException{
        ColorSensor Lcolor = hardwareMap.colorSensor.get("lcolor");
        Lcolor.setI2cAddress(I2cAddr.create8bit(0x2c));
        Lcolor.enableLed(false);
        ColorSensor Rcolor = hardwareMap.colorSensor.get("rcolor");
        Rcolor.setI2cAddress(I2cAddr.create8bit(0x3c));
        Rcolor.enableLed(false);
        sleep(50);

        boolean loop = true;
        while (opModeIsActive()&&loop) {
            if (Rcolor.red() > Rcolor.blue()) {
                telemetry.addData("r red", Rcolor.red());
                telemetry.addData("r blue", Rcolor.blue());
                telemetry.addData("l red", Lcolor.red());
                telemetry.addData("l blue", Lcolor.blue());
                telemetry.addData("red > blue", "");
                telemetry.update();
                Right_Sideways(100, 0.3);
                Reverse(50, 0.4);
                Forward(15, 0.5);
                loop = false;
            } else {
                if (Rcolor.red() < Rcolor.blue()) {
                    telemetry.addData("r red", Rcolor.red());
                    telemetry.addData("r blue", Rcolor.blue());
                    telemetry.addData("l red", Lcolor.red());
                    telemetry.addData("l blue", Lcolor.blue());
                    telemetry.addData("red < blue", "");
                    telemetry.update();
                    Right_Sideways(40, 0.3);
                    Reverse(50, 0.3);
                    Forward(15, 0.5);
                    Right_Sideways(60, 0.2);
                    loop = false;
                } else {
                    telemetry.addData("IK LEES GEEN BEACON", "");
                    telemetry.update();
                    Reverse(1, 0.2);
                }
            }
            telemetry.addLine("push");
            telemetry.addData("r red", Rcolor.red());
            telemetry.addData("r blue", Rcolor.blue());
            telemetry.addData("l red", Lcolor.red());
            telemetry.addData("l blue", Lcolor.blue());
            telemetry.update();
        }
    }
    private void Push2() throws InterruptedException{
        ColorSensor Lcolor = hardwareMap.colorSensor.get("lcolor");
        Lcolor.setI2cAddress(I2cAddr.create8bit(0x2c));
        Lcolor.enableLed(false);
        ColorSensor Rcolor = hardwareMap.colorSensor.get("rcolor");
        Rcolor.setI2cAddress(I2cAddr.create8bit(0x3c));
        Rcolor.enableLed(false);
        sleep(50);

        boolean loop = true;
        while (opModeIsActive()&&loop) {
            if (Rcolor.red() > Rcolor.blue()) {
                telemetry.addData("r red", Rcolor.red());
                telemetry.addData("r blue", Rcolor.blue());
                telemetry.addData("l red", Lcolor.red());
                telemetry.addData("l blue", Lcolor.blue());
                telemetry.addData("red > blue", "");
                telemetry.update();
                Left_Sideways(40, 0.3);
                Reverse(60, 0.4);
                Forward(20, 0.5);
                loop = false;
            } else {
                if (Rcolor.red() < Rcolor.blue()) {
                    telemetry.addData("r red", Rcolor.red());
                    telemetry.addData("r blue", Rcolor.blue());
                    telemetry.addData("l red", Lcolor.red());
                    telemetry.addData("l blue", Lcolor.blue());
                    telemetry.addData("red < blue", "");
                    telemetry.update();
                    Right_Sideways(40, 0.3);
                    Reverse(60, 0.4);
                    Forward(20 ,0.5);
                    loop = false;
                } else {
                    telemetry.addData("IK LEES GEEN BEACON", "");
                    telemetry.update();
                    Reverse(1, 0.2);
                }
            }
            telemetry.addLine("push");
            telemetry.addData("r red", Rcolor.red());
            telemetry.addData("r blue", Rcolor.blue());
            telemetry.addData("l red", Lcolor.red());
            telemetry.addData("l blue", Lcolor.blue());
            telemetry.update();
        }
    }

    private void FollowWallLeft(double omw, double pwr, double pwrmultiplier, double afstand, double threshold) throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");

        UltrasonicSensor ultraR = hardwareMap.ultrasonicSensor.get("ultraR");
        UltrasonicSensor ultraL = hardwareMap.ultrasonicSensor.get("ultraL");

        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);


        double FollowPowerFront = 0;
        double FollowPowerTurn = 0;
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
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (loop && opModeIsActive()){
            telemetry.addLine("followwallleft 1");
            telemetry.addData("Rultrasonic", ultraR.getUltrasonicLevel());
            telemetry.addData("Lultrasonic", ultraL.getUltrasonicLevel());
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", -LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", -RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.addData("data", (LFdrive.getCurrentPosition() - LBdrive.getCurrentPosition() - RFdrive.getCurrentPosition() + RBdrive.getCurrentPosition()) );
            telemetry.update();


            double tmp = gyro.getHeading();
            if (tmp > 271) FollowPowerTurn = -0.16;
            if (tmp < 271) FollowPowerTurn = 0.16;
            if (tmp == 271) FollowPowerTurn = 0;

            //FollowPowerFront = 0;
            //FollowPowerTurn = 0;

            if (((LFdrive.getCurrentPosition() - LBdrive.getCurrentPosition() - RFdrive.getCurrentPosition() + RBdrive.getCurrentPosition()) / 4) > omw * 11.20){
                loop = false;
                telemetry.addData("loop is false", "");
                telemetry.update();
            }

            LFdrive.setPower(pwr - FollowPowerTurn);
            LBdrive.setPower(-pwr - FollowPowerTurn);
            RFdrive.setPower(-pwr + FollowPowerTurn);
            RBdrive.setPower(pwr + FollowPowerTurn);
        }

        while (Flight.getRawLightDetected() < threshold && Blight.getRawLightDetected() < threshold && opModeIsActive()){


            telemetry.addLine("follwallleft 2");
            telemetry.addData("Rultrasonic", ultraR.getUltrasonicLevel());
            telemetry.addData("Lultrasonic", ultraL.getUltrasonicLevel());
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (ultraL.getUltrasonicLevel() > afstand ) FollowPowerFront = -0.1;
            if (ultraL.getUltrasonicLevel() < afstand ) FollowPowerFront = 0.1;

            double tmp = gyro.getHeading();
            if (tmp > 271) FollowPowerTurn = -0.16;
            if (tmp < 271) FollowPowerTurn = 0.16;
            if (tmp == 271) FollowPowerTurn = 0;

            LFdrive.setPower(pwr * pwrmultiplier + (FollowPowerFront - FollowPowerTurn));
            LBdrive.setPower(-pwr * pwrmultiplier + (FollowPowerFront - FollowPowerTurn));
            RFdrive.setPower(-pwr * pwrmultiplier + (FollowPowerFront + FollowPowerTurn));
            RBdrive.setPower(pwr * pwrmultiplier + (FollowPowerFront + FollowPowerTurn));
        }


        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

    }
    private void DriveToLineRight(double pwr, double threshold) throws InterruptedException{
        boolean loop = true;

        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");
        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
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

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (loop && opModeIsActive()){
            telemetry.addLine("drivetolineright");
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.addData("Blight", Blight.getRawLightDetected());
            telemetry.addData("Flight", Flight.getRawLightDetected());
            telemetry.update();

            double degrees = 270;
            double aanpassing = 0.18;
            if (gyro.getHeading() > degrees){
                LFdrive.setPower(pwr-aanpassing);
                LBdrive.setPower(pwr-aanpassing);
                RFdrive.setPower(pwr+aanpassing);
                RBdrive.setPower(pwr+aanpassing);
            }
            if (gyro.getHeading() < degrees){
                LFdrive.setPower(pwr+aanpassing);
                LBdrive.setPower(pwr+aanpassing);
                RFdrive.setPower(pwr-aanpassing);
                RBdrive.setPower(pwr-aanpassing);
            }
            if (gyro.getHeading() == degrees){
                LFdrive.setPower(pwr);
                LBdrive.setPower(pwr);
                RFdrive.setPower(pwr);
                RBdrive.setPower(pwr);
            }
            if (Blight.getRawLightDetected() > threshold || Flight.getRawLightDetected() > threshold){
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    @Override public void runOpMode() throws InterruptedException {

        waitForStart();


        DcMotor intakeR = hardwareMap.dcMotor.get("intaker");
        DcMotor intakeL = hardwareMap.dcMotor.get("intakel");

        Reverse(130, 0.1);

        intakeR.setPower(-0.5);
        intakeL.setPower(0.5);
        sleep(1000);
        intakeL.setPower(0);
        intakeR.setPower(0);

        Forward(30, 0.1);
    }
}
