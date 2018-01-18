package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "VuforiaTelemetryLocation")
@Disabled
public class VuforiaTelemetryLocation extends LinearOpMode
{
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target_Wheels;
    private VuforiaTrackable target_Gears;
    private VuforiaTrackable target_Legos;
    private VuforiaTrackable target_Tools;
    private VuforiaTrackableDefaultListener listener_Wheels;
    private VuforiaTrackableDefaultListener listener_Gears;

    private VuforiaTrackableDefaultListener listener_Legos;
    private VuforiaTrackableDefaultListener listener_Tools;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "AWQ0Ulb/////AAAAGUV8ySxRVUpulp7a+YCjrmlZm2JeXuurkO07bfKUBkHpzLT+YSpn4CoCa0npzqKJIpaEN/8tZNkb7O2n+9SVsHg8J3Ym3QOfQecp6DZ7Tui8+Xmn7LS8nHhTPF7Uu3ghmXYh3GGw8PRgBH/+Imr+zIeiIzMr7zB86pPpXcgJ+6tsGG1gJSfbnQrNkLXAmFNjD5ukobtg6JQs6yaOvHUcRXNCtV1XZVswFY130QGadPmiwiHNKABotOkGuxEb1ql2e8pwRBgjTh35o7tPH0OUGtsMawk/y2Ad2K0hkzVQKZDLmwAnAfb012tcR+8aXTHDQXEcZJ73z3yYWOFl4Jdvox4FooGttkJ30LORvX9gxJFS"; // Insert your own key here

    private float robotX = 0;
    private float robotY = 0;

    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        // X Y Z U V W
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);




        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while(opModeIsActive())
        {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation_Gears = listener_Gears.getUpdatedRobotLocation();
            OpenGLMatrix latestLocation_Legos = listener_Legos.getUpdatedRobotLocation();
            OpenGLMatrix latestLocation_Tools = listener_Tools.getUpdatedRobotLocation();
            OpenGLMatrix latestLocation_Wheels = listener_Wheels.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation_Gears != null) {
                lastKnownLocation = latestLocation_Gears;
            }
            if(latestLocation_Legos != null) {
                lastKnownLocation = latestLocation_Legos;
            }
            if(latestLocation_Tools != null) {
                lastKnownLocation = latestLocation_Tools;
            }
            if(latestLocation_Wheels != null) {
                lastKnownLocation = latestLocation_Wheels;
            }

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the target is visible, and where the robot is
            //telemetry.addData("Tracking " + target_Gears.getName(), listener_Gears.isVisible());
            //telemetry.addData("Tracking " + target_Tools.getName(), listener_Tools.isVisible());
            //telemetry.addData("Tracking " + target_Legos.getName(), listener_Legos.isVisible());
            //telemetry.addData("Tracking " + target_Wheels.getName(), listener_Wheels.isVisible());
            telemetry.addData("robotX" , robotX);
            telemetry.addData("robotY", robotY);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
    }

    private void setupVuforia()
    {
        final float MM_PER_INCH = 25.4f;
        final float ROBOT_WIDTH = 18*MM_PER_INCH;               // in mm
        final float FTC_FIELD_WIDTH = (12*12 - 2)*MM_PER_INCH;  // in mm
        final float TARGET_HEIGHT = 160.0f;                     // in mm

        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target_Wheels = visionTargets.get(0); // 0 corresponds to the wheels target
        target_Wheels.setName("Wheels");
        target_Wheels.setLocation(createMatrix(90.0f, 0.0f, 0.0f, 12.0f*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT));

        target_Tools = visionTargets.get(1);
        target_Tools.setName("Tools");
        target_Tools.setLocation(createMatrix(0.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, 30.0f*MM_PER_INCH, TARGET_HEIGHT));

        target_Legos = visionTargets.get(2);
        target_Legos.setName("Legos");
        target_Legos.setLocation(createMatrix(90.0f, 0.0f, 0.0f, -30.0f*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT));

        target_Gears = visionTargets.get(3);
        target_Gears.setName("Gears");
        target_Gears.setLocation(createMatrix(90.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, -12.0f*MM_PER_INCH, TARGET_HEIGHT));

        // Set phone location on robot
        phoneLocation = createMatrix(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH/2.0f, 0.0f);

        // Setup listener and inform it of phone information
        listener_Wheels = (VuforiaTrackableDefaultListener) target_Wheels.getListener();
        listener_Tools = (VuforiaTrackableDefaultListener) target_Tools.getListener();
        listener_Legos = (VuforiaTrackableDefaultListener) target_Legos.getListener();
        listener_Gears = (VuforiaTrackableDefaultListener) target_Gears.getListener();
        listener_Wheels.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener_Tools.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener_Legos.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener_Gears.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
