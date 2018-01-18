package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class AutonomousVoids extends LinearOpMode  {
    public void Forward(double omw, double pwr) {
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

    public void Reverse(double omw, double pwr) {
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


    public void Left_Sideways(double omw, double pwr)  {
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

    public void Right_Sideways(double omw, double pwr) throws InterruptedException {
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
        RFdrive.setPower(-pwr);
        RBdrive.setPower(-pwr);

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
                LFdrive.setPower(pwr);
            }
            if (LFdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr);
            }


            if (LBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr);
            }
            if (LBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr);
            }


            if (RBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr);
            }
            if (RBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr);
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

    public void TurnRight(double omw, double pwr) throws InterruptedException {
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
                RFdrive.setPower(-pwr * 1.33);
                RBdrive.setPower(-pwr * 1.33);
            }
            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.33);
                LBdrive.setPower(pwr * 1.33);
                RFdrive.setPower(-pwr * 0.75);
                RBdrive.setPower(-pwr * 0.75);
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

    public void TurnLeft(double omw, double pwr) throws InterruptedException {
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
                LFdrive.setPower(-pwr * 0.75);
                LBdrive.setPower(-pwr * 0.75);
                RFdrive.setPower(pwr * 1.33);
                RBdrive.setPower(pwr * 1.33);
            }
            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(-pwr * 1.33);
                LBdrive.setPower(-pwr * 1.33);
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






    @Override
    public void runOpMode() throws InterruptedException {

    }
}
