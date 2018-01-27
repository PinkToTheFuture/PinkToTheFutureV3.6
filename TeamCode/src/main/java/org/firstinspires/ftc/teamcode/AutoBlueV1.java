package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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


@Autonomous(name="AutoBlueV1", group ="Concept")

public class AutoBlueV1 extends LinearOpMode {

    private JewelDetector jewelDetector = null;
    private Pictographs pictographs = Pictographs.CENTER;
    public static final String TAG = "Vuforia VuMark Sample";
    ClosableVuforiaLocalizer vuforia;
    AutonomousVoids voids = new AutonomousVoids();
    DemoRobot1 drive = new DemoRobot1();
    VuforiaTrackable relicTemplate;
    GlyphScoreCenter glyphScoreCenter = new GlyphScoreCenter();

    private CryptoboxDetector cryptoboxDetector = null;
    private GlyphDetector glyphDetector = null;

    public enum Pictographs {
        LEFT, CENTER, RIGHT
    }


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
                    jewelDetector.disable();
                    loop = false;
            }
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        Vumark();
        Jewels();
        if (pictographs == Pictographs.LEFT) {

        }
        if (pictographs == Pictographs.CENTER) {
            //glyphScoreCenter.Score();
        }
        if (pictographs == Pictographs.RIGHT) {

        }
        telemetry.update();
        sleep(5000);
        drive.runOpMode();



    }

}