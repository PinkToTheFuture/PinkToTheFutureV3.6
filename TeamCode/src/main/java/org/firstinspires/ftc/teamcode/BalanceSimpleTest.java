package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

@TeleOp(name = "BalanceSimpleTest", group = "TeleOp")
public class BalanceSimpleTest extends LinearOpMode {
    //Motors
    private DcMotor LFdrive;
    private DcMotor LBdrive;
    private DcMotor RFdrive;
    private DcMotor RBdrive;

    //Sensors
    bno055driver imu;


    @Override
    public void runOpMode() throws InterruptedException {
        LFdrive = hardwareMap.dcMotor.get("LFdrive");
        RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LBdrive = hardwareMap.dcMotor.get("LBdrive");
        RBdrive = hardwareMap.dcMotor.get("RBdrive");

        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new bno055driver("IMU", hardwareMap);

        double Pitchv = 17;     //pitch value (change to 1 to test PID)
        double Rollv = 10;     //roll value (change to 1 to test PID)

        double LFpower = 0;
        double RFpower = 0;
        double LBpower = 0;
        double RBpower = 0;

        //The PID controlller:

        double Kp1 = 1;
        double Ki1 = 1;
        double Kd1 = 1;

        double Kp2 = 1;
        double Ki2 = 1;
        double Kd2 = 1;

        double error1;
        double error2;

        double Sp = 0;
        double Pv1 = imu.getAngles()[1];
        double Pv2 = imu.getAngles()[2];

        double integral1 = 0;
        double integral2 = 0;
        double derivative1;
        double derivative2;


        error1 = Sp - Pv1;
        error2 = Sp - Pv2;

        integral1 = error1 + integral1;

        double lasterror1 = error1;
        double lasterror2 = error2;

        derivative1 = error1 - lasterror1;
        derivative2 = error2 - lasterror2;

        double correction1; //value for the motors from the pitch value
        double correction2; //value for the motors from the roll value

        correction1 = Kp1*error1 + Ki1*integral1 + Kd1*derivative1;
        correction2 = Kp2*error2 + Ki2*integral2 + Kd2*derivative2;

        //End of PID controller

        telemetry.addLine("ready to start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {


            RFpower = ((((-correction1/Pitchv) + (correction2/Rollv)) / 2));
            RBpower = ((((-correction1/Pitchv) - (correction2/Rollv)) / 2));
            LFpower = ((((-correction1/Pitchv) - (correction2/Rollv)) / 2));
            LBpower = ((((-correction2/Pitchv) + (correction2/Rollv)) / 2));

            //RIGHT STICK

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);

            LFdrive.setPower(LFpower);
            RFdrive.setPower(RFpower);
            LBdrive.setPower(LBpower);
            RBdrive.setPower(RBpower);

            telemetry.addLine("Pitch: " + (imu.getAngles()[1])); //pitch
            telemetry.addLine("Roll: " + (imu.getAngles()[2]));  //roll
            telemetry.update();


            sleep(20);
        }
    }

}
