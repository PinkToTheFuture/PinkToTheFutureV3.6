package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by seb7 on 1/14/2018.
 */

public class GlyphScoreCenter extends LinearOpMode {

    AutonomousVoids Autovoids = new AutonomousVoids();
    private CryptoboxDetector cryptoboxDetector = null;
    private GlyphDetector glyphDetector = null;

    public void Cryptobox() throws InterruptedException {

        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        cryptoboxDetector.downScaleFactor = .9;
        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.SLOW;
        cryptoboxDetector.rotateMat = true;

        //Optional Test Code to load images via Drawables
        //cryptoboxDetector.useImportedImage = true;
        //cryptoboxDetector.SetTestMat(com.qualcomm.ftcrobotcontroller.R.drawable.test_cv4);

        cryptoboxDetector.enable();

    }





    @Override
    public void runOpMode() throws InterruptedException {

    }
}

