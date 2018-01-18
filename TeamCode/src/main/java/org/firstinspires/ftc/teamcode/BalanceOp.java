package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "BalanceOp", group = "TeleOp")
public class BalanceOp extends LinearOpMode {
    //Motors
    private DcMotor LFdrive;
    private DcMotor LBdrive;
    private DcMotor RFdrive;
    private DcMotor RBdrive;

    //Sensors
    bno055driver imu;


    @Override
    public void runOpMode() throws InterruptedException {
        //Init everything
        runInitSequence();

        //check if encoders are working
        telemetry.addLine("Enc: " + LBdrive.getCurrentPosition());
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            pitchAndRollBalance(.001, .005);
        }
    }

    //change Kp and Tp depending on your robot
    public double[] rollBalance(double Tp, double Kp) {
        double error = imu.getAngles()[2]; //roll

        double leftPow = error * Kp;
        double rightPow = error * Kp;

        return new double[]{Range.clip(leftPow, -Tp, Tp), Range.clip(rightPow, -Tp, Tp)};
    }

    private void pitchAndRollBalance(double Tp, double Kp) {
        double error;

        double LFpower = 0;
        double RFpower = 0;
        double LBpower = 0;
        double RBpower = 0;

        double[] roll = null;

        double target;

        while (opModeIsActive()) {

            error = imu.getAngles()[1]; //pitch

            roll = rollBalance(.001, .005);

            LFpower = error * Kp;
            RFpower = error * Kp;
            LBpower = error * Kp;
            RBpower = error * Kp;

            LFpower = Range.clip(LFpower, -Tp, Tp) - roll[0];
            RFpower = Range.clip(RFpower, -Tp, Tp) + roll[1];
            LBpower = Range.clip(LBpower, -Tp, Tp) + roll[0];
            RBpower = Range.clip(RBpower, -Tp, Tp) - roll[1];

            LFdrive.setPower(LFpower);
            RFdrive.setPower(RFpower);
            LBdrive.setPower(LBpower);
            RBdrive.setPower(RBpower);

            telemetry.addLine("Pitch: " + (imu.getAngles()[1])); //pitch
            telemetry.addLine("Roll: " + (imu.getAngles()[2]));  //roll
            telemetry.update();
        }

        setLeftSidePower(0);
        setRightSidePower(0);
    }

    private void runInitSequence() {
        //Drive chain motors
        LFdrive = hardwareMap.dcMotor.get("LFdrive");
        RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LBdrive = hardwareMap.dcMotor.get("LBdrive");
        RBdrive = hardwareMap.dcMotor.get("RBdrive");

        //-----------------------------------------------------------------------------

        //Account for reversed motors; they need to be the inverse of what they are in TeleOp because for some stupid reason the XY values from the gamepads are inveresed...
        RFdrive.setDirection(DcMotor.Direction.REVERSE);
        RBdrive.setDirection(DcMotor.Direction.REVERSE);


        //Run without internal PID ---------------------------------------------

        //Drive Chain
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //-----------------------------------------------------------------------

        //Sensors
        imu = new bno055driver("IMU", hardwareMap);
    }

    private void setLeftSidePower(double pow) {
        LBdrive.setPower(pow);
        LFdrive.setPower(pow);
    }

    private void setRightSidePower(double pow) {
        RBdrive.setPower(pow);
        RFdrive.setPower(pow);
    }
}
