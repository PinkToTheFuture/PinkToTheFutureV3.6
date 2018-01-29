package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="Full Robot", group="PinktotheFuture")

public class FullRobotPTTF extends LinearOpMode implements org.firstinspires.ftc.teamcode.ServoVariables {

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower;
        double LBpower;
        double RFpower;
        double RBpower;
        double fastency = 1;
        double geleiderPower = 0;
        double sweeperPower = 0;
        int shooterPosition = 0;

        Servo Armservo = hardwareMap.servo.get("servoarm");
        Armservo.setPosition(ArmservoStopPosition);
        Servo releaseArmL = hardwareMap.servo.get("releasearmL");
        Servo releaseArmR = hardwareMap.servo.get("releasearmR");
        releaseArmL.setPosition(releaseArmLStartPosition);
        releaseArmR.setPosition(releaseArmRStartPosition);
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");
        shooterservoX.setPosition(shooterservoXStartPosition);


        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor sweeper = hardwareMap.dcMotor.get("sweeper");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        DcMotor geleider1 = hardwareMap.dcMotor.get("geleider1");
        DcMotor geleider2 = hardwareMap.dcMotor.get("geleider2");


        TouchSensor shootertouch = hardwareMap.touchSensor.get("shootertouch");

        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        geleider1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        idle();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setTargetPosition(0);
        shooter.setPower(-1);



        waitOneFullHardwareCycle();
        waitForStart();
        while (opModeIsActive()) {
            waitOneFullHardwareCycle();
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.4;

            geleiderPower = 0;
            sweeperPower = 0;

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

            if (fastency < 0.7){
                RFpower = RFpower - (gamepad1.right_stick_x * 0.2);
                RBpower = RBpower - (gamepad1.right_stick_x * 0.2);
                LFpower = LFpower + (gamepad1.right_stick_x * 0.2);
                LBpower = LBpower + (gamepad1.right_stick_x * 0.2);
            }




            if (gamepad2.left_trigger > 0.2){
                sweeperPower = -gamepad2.left_trigger;
            }
            if (gamepad2.right_trigger > 0.2){
                sweeperPower = gamepad2.right_trigger;
            }

            if (gamepad2.dpad_down) {
                releaseArmL.setPosition(releaseArmLEngagePosition);
                releaseArmR.setPosition(releaseArmREngagePosition);
            }
            
            if (gamepad2.dpad_up) {
                releaseArmL.setPosition(releaseArmLStartPosition);
                releaseArmR.setPosition(releaseArmRStartPosition);
            }



            if (gamepad2.b) {
                Armservo.setPosition(ArmservoForwardPosition);
            } else {
                if (gamepad2.x){
                    Armservo.setPosition(ArmservoBackPosition);
                } else {
                    Armservo.setPosition(ArmservoStopPosition);
                }
            }

            if (gamepad2.left_bumper){
                shooterservoX.setPosition(0.15);

                double tijdservox = getRuntime() + 0.5;
                boolean loop = true;
                while (opModeIsActive() && loop){

                    if (getRuntime() > tijdservox){
                        if (shootertouch.isPressed()){
                            loop = false;
                            sleep(140);
                        }
                    }
                    RFpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);
                    RBpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
                    LFpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
                    LBpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);

                    RFpower = RFpower - (gamepad1.right_stick_x);
                    RBpower = RBpower - (gamepad1.right_stick_x);
                    LFpower = LFpower + (gamepad1.right_stick_x);
                    LBpower = LBpower + (gamepad1.right_stick_x);

                    Range.clip(RFpower, -1, 1);
                    Range.clip(RBpower, -1, 1);
                    Range.clip(LFpower, -1, 1);
                    Range.clip(LBpower, -1, 1);
                    LFdrive.setPower(LFpower * fastency);
                    RBdrive.setPower(RBpower * fastency);
                    LBdrive.setPower(LBpower * fastency);
                    RFdrive.setPower(RFpower * fastency);

                }
                shooterservoX.setPosition(0.5);
            }
            if (gamepad2.right_bumper){
                shooterservoX.setPosition(0.7);
            } else {
                shooterservoX.setPosition(0.5);
            }


            if (gamepad1.right_bumper) {
                shooterPosition = shooterPosition + 2240;
                shooter.setTargetPosition(shooterPosition);
                while (opModeIsActive() && shooter.getCurrentPosition() < shooterPosition - 100){
                    idle();
                }

                shooterservoX.setPosition(0.15);
                double tijdservox = getRuntime() + 0.5;
                boolean loop = true;
                while (opModeIsActive() && loop){
                    if (getRuntime() > tijdservox){
                        if (shootertouch.isPressed()){
                            loop = false;
                            sleep(140);
                        }
                    }
                }
                shooterservoX.setPosition(0.5);
            }

            if (gamepad1.dpad_right) {
                shooterPosition = shooterPosition-80;
                shooter.setTargetPosition(shooterPosition);
                while (gamepad1.dpad_right){
                    idle();
                }
            }
            if (gamepad1.dpad_left){
                shooterPosition = shooterPosition+80;
                shooter.setTargetPosition(shooterPosition);
                while (gamepad1.dpad_left){
                    idle();
                }
            }



            if (gamepad2.y){
                geleiderPower = 1;
            }
            if (gamepad2.a) {
                geleiderPower = -0.3;
            }


            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);
            LFdrive.setPower(LFpower * fastency);
            RBdrive.setPower(RBpower * fastency);
            LBdrive.setPower(LBpower * fastency);
            RFdrive.setPower(RFpower * fastency);
            geleider1.setPower(geleiderPower);
            geleider2.setPower(geleiderPower);
            sweeper.setPower(sweeperPower);


            telemetry.addData("runtime:", getRuntime());
            telemetry.addData("shooter encoder", shooter.getCurrentPosition());
            telemetry.addData("shootertouch", shootertouch.isPressed());
            telemetry.addData("shooterPosition", shooterPosition);
            telemetry.addData("shots fired:", shooterPosition / 2240);
            telemetry.update();

        }
        shooter.setPower(0);
    }
}
