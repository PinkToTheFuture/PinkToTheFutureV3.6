package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

import java.util.Arrays;

import static java.lang.String.format;

/**
 * Created by Robotica on 26-9-2016.
 */

@Autonomous(name="VuforiaOp", group="PinktotheFuture")
@Disabled
public class VuforiaProcessingImage extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AWQ0Ulb/////AAAAGUV8ySxRVUpulp7a+YCjrmlZm2JeXuurkO07bfKUBkHpzLT+YSpn4CoCa0npzqKJIpaEN/8tZNkb7O2n+9SVsHg8J3Ym3QOfQecp6DZ7Tui8+Xmn7LS8nHhTPF7Uu3ghmXYh3GGw8PRgBH/+Imr+zIeiIzMr7zB86pPpXcgJ+6tsGG1gJSfbnQrNkLXAmFNjD5ukobtg6JQs6yaOvHUcRXNCtV1XZVswFY130QGadPmiwiHNKABotOkGuxEb1ql2e8pwRBgjTh35o7tPH0OUGtsMawk/y2Ad2K0hkzVQKZDLmwAnAfb012tcR+8aXTHDQXEcZJ73z3yYWOFl4Jdvox4FooGttkJ30LORvX9gxJFS";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizerImplSubclass vuforia = new VuforiaLocalizerImplSubclass(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(0).setName("Tools");
        beacons.get(0).setName("Lego");
        beacons.get(0).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()) ;
        {
            if (vuforia.rgb != null) {
                Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());
            }


            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if (pose != null) {
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);
                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, -92, 0));
                    Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, -92, 0));


                }

            }

        }

    }



}
