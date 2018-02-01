package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Opendag robot zonder fcd", group="PinktotheFuture")
public class Drive_Robot_Omni extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double fastency = 1;
        double geleiderPower = 0;
        double sweeperPower = 0;

        double relicpower = 0;
        double relicposition1 = 0.5;



        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");

        Servo Flapperg1 = hardwareMap.servo.get("flapperg1");
        Servo Flapperg2 = hardwareMap.servo.get("flapperg2");
        DcMotor Geleider = hardwareMap.dcMotor.get("geleider");
        Geleider.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor Intakes1 = hardwareMap.dcMotor.get("intakes1");
        DcMotor Intakes2 = hardwareMap.dcMotor.get("intakes2");
        Intakes2.setDirection(DcMotorSimple.Direction.REVERSE);

        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.3;
            sweeperPower = 0;
            geleiderPower = 0;
            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;


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

            if (gamepad2.dpad_up) {
                relicposition1 = 0.1;
            }
            if (gamepad2.dpad_down) {
                relicposition1 = 1;
            }


            Flapperg1.setPosition(relicposition1);
            Flapperg2.setPosition(-relicposition1 + 1);

            Geleider.setPower(-gamepad2.right_stick_y);


            if (gamepad2.right_trigger > 0){
                Intakes1.setPower(gamepad2.right_trigger);
                Intakes2.setPower(gamepad2.right_trigger);
            } else {

            if (gamepad2.left_trigger > 0){
                Intakes1.setPower(-gamepad2.left_trigger);
                Intakes2.setPower(-gamepad2.left_trigger);
            } else {    Intakes1.setPower(0);
                        Intakes2.setPower(0);}
            }


            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);



            LFdrive.setPower(-LFpower * fastency);
            RBdrive.setPower(-RBpower * fastency);
            LBdrive.setPower(-LBpower * fastency);
            RFdrive.setPower(-RFpower * fastency);

            telemetry.addData("LB",LBpower);
            telemetry.addData("LF",LFpower);
            telemetry.addData("RB",RBpower);
            telemetry.addData("RF",RFpower);
            telemetry.update();


        }
    }
}
