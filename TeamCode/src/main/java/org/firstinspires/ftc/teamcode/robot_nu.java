package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="robot nu helemaal", group="PinktotheFuture")
public class robot_nu extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double fastency = 1;
        double intakefastency = 1;
        double relicpos = 0.4;

        DigitalChannel endbottombak = hardwareMap.get(DigitalChannel.class, "endbottombak");
        DigitalChannel endtopbak = hardwareMap.get(DigitalChannel.class, "endtopbak");
        endbottombak.setMode(DigitalChannel.Mode.INPUT);

        Servo moverelic = hardwareMap.servo.get("moverelic");
        Servo grabrelic = hardwareMap.servo.get("grabrelic");
        Servo bakjeturn = hardwareMap.servo.get("bakjeturn");
        Servo bakjedicht = hardwareMap.servo.get("bakjedicht");

        DcMotor intakeR = hardwareMap.dcMotor.get("intaker");
        DcMotor intakeL = hardwareMap.dcMotor.get("intakel");
        DcMotor relic = hardwareMap.dcMotor.get("relic");
        relic.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor bak = hardwareMap.dcMotor.get("bak");

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");

        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        RFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.3;
            if (gamepad1.dpad_left)   fastency = 0.6;


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


            RFpower = RFpower + (gamepad1.right_stick_x);
            RBpower = RBpower + (gamepad1.right_stick_x);
            LFpower = LFpower - (gamepad1.right_stick_x);
            LBpower = LBpower - (gamepad1.right_stick_x);

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);



            LFdrive.setPower(-LFpower * fastency);
            RBdrive.setPower(-RBpower * fastency);
            LBdrive.setPower(-LBpower * fastency);
            RFdrive.setPower(-RFpower * fastency);





            if (gamepad2.left_bumper)  grabrelic.setPosition(0);
            if (gamepad2.right_bumper)   grabrelic.setPosition(0.5);


            if (gamepad2.dpad_up)  relicpos -= 0.005;
            if (gamepad2.dpad_down)  relicpos += 0.005;
            moverelic.setPosition(relicpos);

            if (gamepad2.a) bakjedicht.setPosition(0.2);
            if (gamepad2.b) bakjedicht.setPosition(0.0);

            if (gamepad2.x) bakjeturn.setPosition(0.3);
            if (gamepad2.y) bakjeturn.setPosition(1);



            if (gamepad2.back){
                intakefastency = 0.5;
            }
            if (gamepad2.start){
                intakefastency = 1;
            }
            if (gamepad1.left_trigger > 0.1){
                intakeL.setPower(gamepad1.right_trigger * intakefastency);
                intakeR.setPower(-gamepad1.right_trigger * intakefastency);
                //intakeR.setPower(((Math.sin(getRuntime()*4) )*0.25 + 0.75) * -gamepad1.left_trigger);
                //intakeL.setPower(((Math.sin(getRuntime()*4) )*0.25 + 0.75) * gamepad1.left_trigger);
                telemetry.addData("ja",intakeL.getPower());
            } else {
                if (gamepad1.right_trigger > 0.1) {
                    intakeL.setPower(-gamepad1.right_trigger * intakefastency);
                    intakeR.setPower(gamepad1.right_trigger * intakefastency);
                } else {
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                }
            }

            if (gamepad2.left_trigger > 0.1) {
                intakeL.setPower(gamepad2.left_trigger * intakefastency);
                intakeR.setPower(-gamepad2.left_trigger * intakefastency);

            } else {
                if (gamepad2.right_trigger > 0.1) {
                    intakeL.setPower(-gamepad2.right_trigger * intakefastency);
                    intakeR.setPower(gamepad2.right_trigger * intakefastency);
                } else {
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                }
            }


            switch (endbottombak.getState() + "-" + endtopbak.getState()){
                case "true-true":
                    telemetry.addData("ERROR","both end stops at the same time");
                    telemetry.update();
                    //stop();

                case "true-false":
                    //bak.setPower(0.2);

                case "false-true":
                    //bak.setPower(-0.2);
                case "false-false":
                    bak.setPower(gamepad2.right_stick_y);
            }


            bak.setPower(gamepad2.right_stick_y);
            relic.setPower(gamepad2.left_stick_y);


            telemetry.addData("LB",LBpower);
            telemetry.addData("LF",LFpower);
            telemetry.addData("RB",RBpower);
            telemetry.addData("RF",RFpower);
            telemetry.update();


        }
    }
}
