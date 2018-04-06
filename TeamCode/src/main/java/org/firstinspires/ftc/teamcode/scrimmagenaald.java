package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="scrimmagenaald V1", group="PinktotheFuture")
public class scrimmagenaald extends LinearOpMode {
    bno055driver imu;


    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double fastency = 1;


        imu = new bno055driver("IMU", hardwareMap);

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

        //RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.3;

            double gyro = imu.getAngles()[0];
            double newZ;

            double K2 = 0;
            double K1 = 0;
            double JoyZ = gamepad1.left_stick_x;

            newZ = JoyZ + K1*(JoyZ-K2*gyro);

            if (newZ>1) {
                newZ =1;
            }
            else if (newZ<-1) {
                newZ = -1;
            }


            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;


            RFpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);
            RBpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
            LFpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
            LBpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);

            //RIGHT STICK
            RFpower = RFpower - (newZ);
            RBpower = RBpower - (newZ);
            LFpower = LFpower + (newZ);
            LBpower = LBpower + (newZ);


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

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);


            if (gamepad2.right_bumper)  grabrelic.setPosition(0);
            if (gamepad2.left_bumper)   grabrelic.setPosition(0.55);

            if (gamepad2.dpad_up)  moverelic.setPosition(0.5);

            if (gamepad2.dpad_down)  moverelic.setPosition(0);

            if (gamepad2.a) bakjedicht.setPosition(0.2);
            if (gamepad2.b) bakjedicht.setPosition(0.0);

            if (gamepad2.x) bakjeturn.setPosition(0);
            if (gamepad2.y) bakjeturn.setPosition(0.5);


            if (gamepad2.left_trigger > 0.2) {
                intakeL.setPower(gamepad2.left_trigger);
                intakeR.setPower(-gamepad2.left_trigger);
            } else {
            if (gamepad2.right_trigger > 0.2) {
                intakeL.setPower(-gamepad2.right_trigger);
                intakeR.setPower(gamepad2.right_trigger);
            } else {
                intakeL.setPower(0);
                intakeR.setPower(0);
            }
            }
            relic.setPower(gamepad2.left_stick_y);
            bak.setPower(gamepad2.right_stick_y);
            LFdrive.setPower(LFpower * fastency);
            RBdrive.setPower(RBpower * fastency);
            LBdrive.setPower(LBpower * fastency);
            RFdrive.setPower(RFpower * fastency);



            telemetry.addData("LB",LBpower);
            telemetry.addData("LF",LFpower);
            telemetry.addData("RB",RBpower);
            telemetry.addData("RF",RFpower);
            telemetry.update();


        }
    }
}
