package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
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
 * Created by seb7 on 3/26/2017.
 */

//Autonomous beta mecanum BLUE with vuforia
@Autonomous(name="VuforiaOmniBlue", group="PinktotheFuture")
@Disabled
public class VuforiaOmniBlue extends LinearOpMode {

    // Variables to be used for later
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

    //void for forward with encoders
    public void Forward(double omw, double pwr) throws InterruptedException{
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");


        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);


        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //omwentelingen invoeren, encoder  ticks uitkrijgen
        double ticks = omw * 1478.4;


        LFdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (LBdrive.getCurrentPosition() < ticks || RBdrive.getCurrentPosition() < ticks && opModeIsActive()){
            waitOneFullHardwareCycle();
            if (LBdrive.getCurrentPosition() > ticks){
                LFdrive.setPower(0);
                LBdrive.setPower(0);
            }
            if (RBdrive.getCurrentPosition() > ticks) {
                RFdrive.setPower(0);
                RBdrive.setPower(0);
            }
            if (RBdrive.getCurrentPosition() < LBdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.7);
                LBdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 1.3);
            }
            if (RBdrive.getCurrentPosition() > LBdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.3);
                LBdrive.setPower(pwr * 1.3);
                RFdrive.setPower(pwr * 0.7);
                RFdrive.setPower(pwr * 0.7);
            }
            telemetry.addData("L", LBdrive.getCurrentPosition());
            telemetry.addData("R", RBdrive.getCurrentPosition());
            telemetry.update();
        }
        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);
    }

    //void setting up vuforia
    public void setupVuforia ()
    {
        final float MM_PER_INCH = 25.4f;
        final float ROBOT_WIDTH = 18*MM_PER_INCH;               // in mm
        final float FTC_FIELD_WIDTH = (12*12 - 2)*MM_PER_INCH;  // in mm
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
        target_Wheels.setLocation(createMatrix(90.0f, 0.0f, 0.0f, 12.0f*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT));

        target_Legos = visionTargets.get(2);
        target_Legos.setName("Legos");
        target_Legos.setLocation(createMatrix(90.0f, 0.0f, 0.0f, -30.0f*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT));

        // Set phone location on robot
        phoneLocation = createMatrix(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH/2.0f, 0.0f);

        // Setup listener and inform it of phone information
        listener_Wheels = (VuforiaTrackableDefaultListener) target_Wheels.getListener();
        listener_Legos = (VuforiaTrackableDefaultListener) target_Legos.getListener();

        listener_Wheels.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener_Legos.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }



    //void for shooter
    public void shoot(double omw, double pwr) throws InterruptedException{
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //omwentelingen invoeren, encoder ticks uitkrijgen
        double  ticks = omw * 1440;

        shooter.setPower(-pwr);

        while (shooter.getCurrentPosition() < ticks && opModeIsActive()){
            waitOneFullHardwareCycle();
            telemetry.addData("shooter", shooter.getCurrentPosition());
            telemetry.update();
        }
        shooter.setPower(0);
    }

    //calculating vectors to beacon
    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float)
                (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle))
                        - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                        trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle))
                        - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));

    }


    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }




    @Override
    public  void runOpMode() throws InterruptedException {

        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        // X Y Z U V W
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);


        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");

        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);


        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo lbs = hardwareMap.servo.get("lbs");
        Servo rbs = hardwareMap.servo.get("rbs");
        Servo shooterservo = hardwareMap.servo.get("shooterservo");
        shooterservo.setPosition(1);
        lbs.setPosition(1);
        rbs.setPosition(0);


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

        //----------------------------------------------------------------------------------------------------------------------------------------------Opmode starts

        Forward(0.5, 0.25);
        sleep(100);
        shoot(1.5, 0.5);
        shooterservo.setPosition(0.5);
        sleep(100);
        shoot(1.5, 0.5);
        sleep(100);


        visionTargets.activate();

        //telemetry of what to the robot sees and where it is
        while(opModeIsActive())
        {
            waitOneFullHardwareCycle();
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation_Legos = listener_Legos.getUpdatedRobotLocation();
            OpenGLMatrix latestLocation_Wheels = listener_Wheels.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation_Legos != null) {
                lastKnownLocation = latestLocation_Legos;
            }
            if(latestLocation_Wheels != null) {
                lastKnownLocation = latestLocation_Wheels;
            }

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target_Legos.getName(), listener_Legos.isVisible());
            telemetry.addData("Tracking " + target_Wheels.getName(), listener_Wheels.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }

        //use drive algorithm to translate coordinates into movement

//        while (opModeIsActive()) {
//            if ((robotY + robotX) > (robotX + robotY)){
//                //X & Y modus
//                //LEFT STICK
//                RFpower = ((robotY - robotX) / 2);
//                RBpower = ((robotY + robotX) / 2);
//                LFpower = ((robotY - robotX) / 2);
//                LBpower = ((robotY + robotX) / 2);
//
//            }
//
//            if ((robotX + robotX) > (robotY + robotX)){
//                //TURN modus
//
//                RFpower = robotX;
//                RBpower = robotX;
//                LFpower = -robotX;
//                LBpower = -robotX;
//            }
//
//            LFdrive.setPower(Range.clip((LFpower * fastency), -1, 1));
//            RBdrive.setPower(Range.clip((RBpower * fastency), -1, 1));
//            LBdrive.setPower(Range.clip((LBpower * fastency), -1, 1));
//            RFdrive.setPower(Range.clip((RFpower * fastency), -1, 1));
//
//
//        algorithm for location to mecanum with heading
//              Front left = -x + y + z
//              Front right: x + y - z
//              Back left = x + y + z
//              Back right = -x + y - z
//
//
//
//
//
//
//           }

        //rijden tot dat hij de beacon ziet
        LFdrive.setPower(0.2);
        RFdrive.setPower(-0.2);
        LBdrive.setPower(-0.2);
        RBdrive.setPower(0.2);
        while (opModeIsActive() && wheels.getRawPose() == null)
        { waitOneFullHardwareCycle();
            idle();
        }
        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);
        sleep(5000);


        //naar beacon rijden
        VectorF angles = anglesFromTarget(wheels);
        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0)); //5cm of the wall

        if (trans.get(0) > 0) {
            //draai links
            LFdrive.setPower(-0.2);
            RFdrive.setPower(0.2);
            LBdrive.setPower(-0.2);
            RBdrive.setPower(0.2);

        } else {
            //draai rechts
            LFdrive.setPower(0.2);
            RFdrive.setPower(-0.2);
            LBdrive.setPower(0.2);
            RBdrive.setPower(-0.2);
        }


        do {
            if (wheels.getPose() != null) {
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0))>30);

        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);


        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive() && LFdrive.isBusy() && RFdrive.isBusy() && LBdrive.isBusy() && RBdrive.isBusy()) {
            waitOneFullHardwareCycle();
            idle();
        }

        LFdrive.setPower(0);
        RFdrive.setPower(0);
        LBdrive.setPower(0);
        RBdrive.setPower(0);

        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10) {
            waitOneFullHardwareCycle();
            if (wheels.getPose() != null) {
                if (Math.abs(wheels.getPose().getTranslation().get(0)) > 0) {
                    LFdrive.setPower(-0.2);
                    RFdrive.setPower(0.2);
                    LBdrive.setPower(-0.2);
                    RBdrive.setPower(0.2);
                } else {
                    LFdrive.setPower(0.2);
                    RFdrive.setPower(-0.2);
                    LBdrive.setPower(0.2);
                    RBdrive.setPower(-0.2);
                }
            } else {
                LFdrive.setPower(-0.2);
                RFdrive.setPower(0.2);
                LBdrive.setPower(-0.2);
                RBdrive.setPower(0.2);

            }

            LFdrive.setPower(0);
            RFdrive.setPower(0);
            LBdrive.setPower(0);
            RBdrive.setPower(0);
            sleep(100);
        }


    }

}



