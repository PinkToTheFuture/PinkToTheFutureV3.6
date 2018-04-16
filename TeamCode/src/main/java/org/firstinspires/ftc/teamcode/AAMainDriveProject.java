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


@TeleOp(name="Main Drive Project", group="PinktotheFuture")
public class AAMainDriveProject extends LinearOpMode {
    bno055driver imu;

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double speed = 1;
        double relicpos = 0.6;
        double relicanglepos = 0.5;
        double timeforbak1 = 0;
        double timeforbak2 = 0;
        boolean bakjebovenpos = false;
        boolean bakjedichtpos = false;
        boolean timeforbakbool1 = false;
        boolean timeforbakbool2 = false;
        boolean ignore2bBak = false;

        DigitalChannel endbottomglyph = hardwareMap.get(DigitalChannel.class, "endbottombak");
        DigitalChannel endtopglyph = hardwareMap.get(DigitalChannel.class, "endtopbak");
        endbottomglyph.setMode(DigitalChannel.Mode.INPUT);
        endtopglyph.setMode(DigitalChannel.Mode.INPUT);

        Servo moverelic = hardwareMap.servo.get("moverelic");
        Servo grabrelic = hardwareMap.servo.get("grabrelic");
        Servo bakjeturn = hardwareMap.servo.get("bakjeturn");
        Servo bakjedicht = hardwareMap.servo.get("bakjedicht");
        Servo anglefishingrod = hardwareMap.servo.get("anglefishingrod");
        Servo filamentmanipulator = hardwareMap.servo.get("filamentmanipulator");
        filamentmanipulator.setPosition(1);
        Servo jewelextender = hardwareMap.servo.get("jewelextender");
        Servo jewelchooser = hardwareMap.servo.get("jewelchooser");

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

        imu = new bno055driver("IMU", hardwareMap);

        Double[] imuArray;
        imuArray = new Double[1];
        imuArray[0] = 0.0;

        waitForStart();


        while (opModeIsActive()) {
            filamentmanipulator.setPosition(0.5 );
            if (gamepad1.dpad_up)     speed = 1;
            if (gamepad1.dpad_down)   speed = 0.3;
            if (gamepad1.dpad_left)   speed = 0.6;

            double temp;

            double theta = imu.getAngles()[0];

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rcw = -gamepad1.right_stick_x;

            if (Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0 || Math.abs(gamepad1.right_stick_y) > 0 ){
                imuArray[0] = theta;
            }

            if (theta > 0) {
                temp = forward*Math.cos(theta)-strafe*Math.sin(theta);
                strafe = forward*Math.sin(theta)+strafe*Math.cos(theta);
                forward = temp;
            }

            if (theta <= 0) {
                temp = forward*Math.cos(theta)-strafe*Math.sin(theta);
                strafe = forward*Math.sin(theta)+strafe*Math.cos(theta);
                forward = temp;
            }

            LFpower = forward+rcw+strafe;
            RFpower = forward-rcw-strafe;
            LBpower = forward+rcw-strafe;
            RBpower = forward-rcw+strafe;

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);

            LFdrive.setPower(-LFpower * speed);
            RBdrive.setPower(-RBpower * speed);
            LBdrive.setPower(-LBpower * speed);
            RFdrive.setPower(-RFpower * speed);



            if (gamepad1.y) jewelextender.setPosition(0);
            if (gamepad1.x) jewelextender.setPosition(0.95);
            if (gamepad1.a && !gamepad1.start) jewelchooser.setPosition(0.3);
            if (gamepad1.b && !gamepad1.start) jewelchooser.setPosition(0.7);

            if (gamepad2.right_bumper)  grabrelic.setPosition(0.13);
            if (gamepad2.left_bumper)   grabrelic.setPosition(0.7);


            if (gamepad2.dpad_up)  relicpos += 0.01;
            if (gamepad2.dpad_down)  relicpos -= 0.01;
            if (gamepad2.dpad_right) relicpos = 0.6;
            moverelic.setPosition(relicpos);



            if (gamepad2.b && !gamepad2.start && !timeforbakbool1 && !timeforbakbool2) {
                timeforbak1 = getRuntime() + 0.5;
                timeforbak2 = getRuntime() + 1.2;
                bakjedicht.setPosition(0.4);
                bakjedichtpos = true;
                timeforbakbool1 = true;
                timeforbakbool2 = true;

            }
            telemetry.addData("boven:", bakjebovenpos);
            telemetry.addData("dicht:", bakjedichtpos);
            if (timeforbak1<getRuntime() && timeforbakbool1){
                timeforbakbool1 = false;
                if (!bakjebovenpos) {
                    bakjeturn.setPosition(0.2);
                    bakjebovenpos = true;
                } else {
                    bakjeturn.setPosition(1);
                    bakjebovenpos = false;
                }
                timeforbakbool2 = true;
            }
            if (timeforbak2<getRuntime() && timeforbakbool2){
                timeforbakbool2 = false;
                if (!bakjebovenpos){
                    bakjedicht.setPosition(0.05);
                    bakjedichtpos = false;
                }
            }

            if (gamepad1.a && !gamepad1.start) {
                bakjeturn.setPosition(0.2);
                bakjedicht.setPosition(0.05);
            }


            relicanglepos = relicanglepos + gamepad2.left_stick_x/30;
            if (relicanglepos > 0.8){
                relicanglepos = 0.8;
            }
            if (relicanglepos < 0.2){
                relicanglepos = 0.2;
            }
            anglefishingrod.setPosition(relicanglepos);

            if (gamepad2.right_trigger > 0.1) {
                intakeL.setPower(gamepad2.right_trigger);
                intakeR.setPower(-gamepad2.right_trigger);

            } else {
                if (gamepad2.left_trigger > 0.1) {
                    intakeL.setPower(-gamepad2.left_trigger);
                    intakeR.setPower(gamepad2.left_trigger);
                } else {
                    intakeR.setPower(0);
                    intakeL.setPower(0);
                }
            }


            switch (endbottomglyph.getState() + "-" + endtopglyph.getState()){
                case "true-true":
                    telemetry.addData("ERROR","both end stops at the same time");
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
