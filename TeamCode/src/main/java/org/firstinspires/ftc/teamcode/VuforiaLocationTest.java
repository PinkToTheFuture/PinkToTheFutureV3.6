package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by seb7 on 3/27/2017.
 */
@Autonomous(name="VuforiaLocationTest", group="PinktotheFuture")
@Disabled
public class VuforiaLocationTest extends LinearOpMode {


    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target_Wheels;
    private VuforiaTrackable target_Legos;
    private VuforiaTrackableDefaultListener listener_Wheels;

    private VuforiaTrackableDefaultListener listener_Legos;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private float robotX = 0;
    private float robotY = 0;

    private float robotAngle = 0;

    public void setupVuforia() {
        final float MM_PER_INCH = 25.4f;
        final float ROBOT_WIDTH = 18 * MM_PER_INCH;               // in mm
        final float FTC_FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;  // in mm
        final float TARGET_HEIGHT = 160.0f;                     // in mm

        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AWQ0Ulb/////AAAAGUV8ySxRVUpulp7a+YCjrmlZm2JeXuurkO07bfKUBkHpzLT+YSpn4CoCa0npzqKJIpaEN/8tZNkb7O2n+9SVsHg8J3Ym3QOfQecp6DZ7Tui8+Xmn7LS8nHhTPF7Uu3ghmXYh3GGw8PRgBH/+Imr+zIeiIzMr7zB86pPpXcgJ+6tsGG1gJSfbnQrNkLXAmFNjD5ukobtg6JQs6yaOvHUcRXNCtV1XZVswFY130QGadPmiwiHNKABotOkGuxEb1ql2e8pwRBgjTh35o7tPH0OUGtsMawk/y2Ad2K0hkzVQKZDLmwAnAfb012tcR+8aXTHDQXEcZJ73z3yYWOFl4Jdvox4FooGttkJ30LORvX9gxJFS";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 2);

        // Setup the target to be tracked
        target_Wheels = visionTargets.get(0); // 0 corresponds to the wheels target
        target_Wheels.setName("Wheels");
        target_Wheels.setLocation(createMatrix(90.0f, 0.0f, 0.0f, 12.0f * MM_PER_INCH, FTC_FIELD_WIDTH / 2.0f, TARGET_HEIGHT));

        target_Legos = visionTargets.get(2);
        target_Legos.setName("Legos");
        target_Legos.setLocation(createMatrix(90.0f, 0.0f, 0.0f, -30.0f * MM_PER_INCH, FTC_FIELD_WIDTH / 2.0f, TARGET_HEIGHT));

        // Set phone location on robot
        phoneLocation = createMatrix(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH / 2.0f, 0.0f);

        // Setup listener and inform it of phone information
        listener_Wheels = (VuforiaTrackableDefaultListener) target_Wheels.getListener();
        listener_Legos = (VuforiaTrackableDefaultListener) target_Legos.getListener();

        listener_Wheels.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener_Legos.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    //create matrix of 6 axes
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    public String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new VectorF((float)
                (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle))
                        - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle))
                - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));

    }


    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image) {
        float[] data = image.getRawPose().getData();
        float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        // X Y Z U V W
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AWQ0Ulb/////AAAAGUV8ySxRVUpulp7a+YCjrmlZm2JeXuurkO07bfKUBkHpzLT+YSpn4CoCa0npzqKJIpaEN/8tZNkb7O2n+9SVsHg8J3Ym3QOfQecp6DZ7Tui8+Xmn7LS8nHhTPF7Uu3ghmXYh3GGw8PRgBH/+Imr+zIeiIzMr7zB86pPpXcgJ+6tsGG1gJSfbnQrNkLXAmFNjD5ukobtg6JQs6yaOvHUcRXNCtV1XZVswFY130QGadPmiwiHNKABotOkGuxEb1ql2e8pwRBgjTh35o7tPH0OUGtsMawk/y2Ad2K0hkzVQKZDLmwAnAfb012tcR+8aXTHDQXEcZJ73z3yYWOFl4Jdvox4FooGttkJ30LORvX9gxJFS";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 2);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Legos");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
        VuforiaTrackableDefaultListener Legos = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();

        waitForStart();

        while (opModeIsActive()) {
            waitOneFullHardwareCycle();
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation_Legos = listener_Legos.getUpdatedRobotLocation();
            OpenGLMatrix latestLocation_Wheels = listener_Wheels.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if (latestLocation_Legos != null) {
                lastKnownLocation = latestLocation_Legos;
            }
            if (latestLocation_Wheels != null) {
                lastKnownLocation = latestLocation_Wheels;
            }

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("robotX" , robotX);
            telemetry.addData("robotY", robotY);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
            telemetry.addData("Tracking " + target_Wheels.getName(), listener_Wheels.isVisible());
            telemetry.addData("Tracking" + target_Legos.getName(), listener_Legos.isVisible());

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }

        while (opModeIsActive() && wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10) {
            waitOneFullHardwareCycle();
            if (wheels.getPose() != null) {
                if (Math.abs(wheels.getPose().getTranslation().get(0)) > 0) {
                    telemetry.addData("links draaien", Math.abs(wheels.getPose().getTranslation().get(0)));
                    telemetry.update();
                } else {
                    telemetry.addLine("rechts draaien");
                    telemetry.update();
                }
            } else {
                telemetry.addLine("rechts draaien");
                telemetry.update();

            }
            idle();
        }
    }
}
