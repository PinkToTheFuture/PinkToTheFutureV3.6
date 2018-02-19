package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.disnodeteam.dogecv.detectors.JewelDetector.JewelDetectionMode.PERFECT_AREA;


@Autonomous(name="OpMode Pink To The Future", group ="Concept")

public class AutoBlueV1 extends LinearOpMode {

    private JewelDetector jewelDetector = null;
    private Pictographs pictographs = Pictographs.CENTER;
    public static final String TAG = "Vuforia VuMark Sample";
    ClosableVuforiaLocalizer vuforia;

    AutonomousVoids Autovoids = new AutonomousVoids();

    VuforiaTrackable relicTemplate;
    GlyphScoreCenter glyphScoreCenter = new GlyphScoreCenter();

    private CryptoboxDetector cryptoboxDetector = null;
    private GlyphDetector glyphDetector = null;

    public enum Pictographs {
        LEFT, CENTER, RIGHT
    }

    bno055driver imu;



    public void Vumark() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //delete cameraMonitorViewId to disable preview

        parameters.vuforiaLicenseKey = "AdQe/E//////AAAAGdUKVJKmNUyWi5sJUoS1G+d+ENxg28Ca11lGwDD6yFPE9hFgVC2x4O0CCFjPJzamT67NyeIzQYo4q0A3z4rJs6h76WVGT8Urwoi2AXXo/awgby8sTLQs8GXzvIg8WuS+7MvCiIKSvEwzv9FBsX8N8trXTsHsdfA7B3LB9C/rScSqDKulPKFTzbdgJvNRGJ8a6S1udF1q6FSZ5UPSFeEYsbQPpC7KBVuFbQAdtxikzobiBfkcHVWkPBJ77dvKkH8bi2tRPpWxqDDo0ZgQH5pTMI7NpKESokFWo8bNFbwvsVv9sK2QPDY8zd2l0Bo+ZOFypY4gdBpFhEiaX9TS/60Ee+LTL/5ExbkahObffUjnCb9X";

        parameters.cameraDirection = ClosableVuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);
        //change closable back (null object reference)


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        relicTrackables.activate();

        boolean loop = true;

        while (opModeIsActive()&& loop) {
            waitOneFullHardwareCycle();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                pictographs = Pictographs.LEFT;
                vuforia.close();
                loop = false;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                pictographs = Pictographs.CENTER;
                vuforia.close();
                loop = false;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                pictographs = Pictographs.RIGHT;
                vuforia.close();
                loop = false;
            }
            telemetry.update();
        }
    }

    public void Jewels() throws InterruptedException {

        Servo Jewelservo = hardwareMap.servo.get("Jewelservo");

        double jewelpos = 0;

        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.05;
        jewelDetector.downScaleFactor = 1;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; //<- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 10;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.rotateMat = true;
        jewelDetector.enable();

        boolean loop = true;
        while (opModeIsActive()&&loop) {
            /*telemetry.addData("Current Order", "Jewel Order: " + jewelDetector.getCurrentOrder().toString()); // Current Result
            telemetry.addData("Last Order", "Jewel Order: " + jewelDetector.getLastOrder().toString()); // Last Known Result */
            waitOneFullHardwareCycle();
            switch (jewelDetector.getLastOrder()) {
                case RED_BLUE:
                    jewelpos = 1;
                    Jewelservo.setPosition(jewelpos);
                    sleep(1000);
                    TurnLeft(50, .3);
                    jewelDetector.disable();
                    loop = false;
            }
        }
    }

    public void Pictographs() throws InterruptedException {
        boolean loop = true;
        while (opModeIsActive()&&loop) {
            if (pictographs == Pictographs.LEFT) {
                loop=false;
            }
            if (pictographs == Pictographs.CENTER) {
                //glyphScoreCenter.Score();
                loop=false;
            }
            if (pictographs == Pictographs.RIGHT) {
                loop=false;
            }
        }
    }

    public void Cryptobox() throws InterruptedException {

        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        cryptoboxDetector.downScaleFactor = .9;
        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.BALANCED;
        cryptoboxDetector.rotateMat = true;

        //Optional Test Code to load images via Drawables
        //cryptoboxDetector.useImportedImage = true;
        //cryptoboxDetector.SetTestMat(com.qualcomm.ftcrobotcontroller.R.drawable.test_cv4);

        cryptoboxDetector.enable();

    }



    public void Teleop() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double fastency = 1;

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");

        //RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new bno055driver("IMU", hardwareMap);

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) fastency = 1;
            if (gamepad1.dpad_right) fastency = .5;
            if (gamepad1.dpad_down) fastency = 0.3;

            double forward = 0;

            double fwd = gamepad1.left_stick_y;
            double stf = gamepad1.left_stick_x;
            double rcw = gamepad1.right_stick_x;

            double gyroyaw = imu.getAngles()[0];
            float temp = (float) (fwd * Math.cos(gyroyaw) + stf * Math.sin(gyroyaw));

            stf = -fwd * Math.sin(gyroyaw) + stf * Math.cos(gyroyaw);
            forward = temp;

            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;


            RFpower = -((forward + stf) / 2);
            RBpower = -((forward - stf) / 2);
            LFpower = -((forward - stf) / 2);
            LBpower = -((forward + stf) / 2);

            //RIGHT STICK
            RFpower = RFpower - (rcw);
            RBpower = RBpower - (rcw);
            LFpower = LFpower + (rcw);
            LBpower = LBpower + (rcw);


            if (stf > -0.1 && stf < 0.1) {
                RFpower = -forward;
                RBpower = -forward;
                LFpower = -forward;
                LBpower = -forward;
            }
            if (forward > -0.1 && forward < 0.1) {
                RFpower = -stf;
                RBpower = stf;
                LFpower = stf;
                LBpower = -stf;
            }

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);


            LFdrive.setPower(LFpower * fastency);
            RBdrive.setPower(RBpower * fastency);
            LBdrive.setPower(LBpower * fastency);
            RFdrive.setPower(RFpower * fastency);


            telemetry.addData("LB", LBpower);
            telemetry.addData("LF", LFpower);
            telemetry.addData("RB", RBpower);
            telemetry.addData("RF", RFpower);
            telemetry.update();
        }
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

        while (loop && opModeIsActive()){

            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);

            LFdrive.setTargetPosition((int) omw * 1120);
            LBdrive.setTargetPosition((int) omw * 1120);
            RFdrive.setTargetPosition((int) omw * 1120);
            RBdrive.setTargetPosition((int) omw * 1120);

            waitOneFullHardwareCycle();
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


        waitForStart();
        Jewels();
        Vumark();
        //Pictographs();

        //Teleop();



    }

}