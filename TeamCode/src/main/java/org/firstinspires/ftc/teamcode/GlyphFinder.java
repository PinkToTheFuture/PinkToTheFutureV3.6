package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="GlyphFinder", group="FTC")


public class GlyphFinder extends OpMode {

    private GlyphDetector glyphDetector = null;

    @Override
    public void init() {

        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = .7;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();
        glyphDetector.rotateMat = true;


    }

    @Override
    public void loop() {
        telemetry.addData("Glyph Pos X", glyphDetector.getChosenGlyphOffset());
        telemetry.addData("Glyph Pos Offest", glyphDetector.getChosenGlyphPosition().toString());


    }
}
