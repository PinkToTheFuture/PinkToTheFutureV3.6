package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Cali Sensor", group = "cali")

public class CaliSensors extends LinearOpMode {

    bno055driver imu2;
    BNO055IMU imu;

    @Override public void runOpMode() throws InterruptedException {
        /*UltrasonicSensor ultraR = hardwareMap.ultrasonicSensor.get("ultraR");
        UltrasonicSensor ultraL = hardwareMap.ultrasonicSensor.get("ultraL");
        ColorSensor Lcolor = hardwareMap.colorSensor.get("lcolor");
        Lcolor.setI2cAddress(I2cAddr.create8bit(0x3c));
        Lcolor.enableLed(false);
        ColorSensor Rcolor = hardwareMap.colorSensor.get("rcolor");
        Rcolor.setI2cAddress(I2cAddr.create8bit(0x2c));
        Rcolor.enableLed(false);

        TouchSensor shootertouch = hardwareMap.touchSensor.get("shootertouch");

        byte[] Lrangesensorcache;
        I2cDevice LrangeSensor = hardwareMap.i2cDevice.get("lrangesensor");
        I2cDeviceSynch Lrangereader;
        Lrangereader = new I2cDeviceSynchImpl(LrangeSensor, I2cAddr.create8bit(0x10), false);
        Lrangereader.engage();


        byte[] Rrangesensorcache;
        I2cDevice RrangeSensor = hardwareMap.i2cDevice.get("rrangesensor");
        I2cDeviceSynch Rrangereader;
        Rrangereader = new I2cDeviceSynchImpl(RrangeSensor, I2cAddr.create8bit(0x28), false);
        Rrangereader.engage();


        ModernRoboticsI2cGyro gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.setI2cAddress(I2cAddr.create8bit(0x20));

        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);


        gyro.calibrate();
        while (gyro.isCalibrating()){
            idle();
        }
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        */

        imu2 = new bno055driver("IMU", hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "IMU");

        waitForStart();



        while (opModeIsActive()) {
            /*Rrangesensorcache = Rrangereader.read(0x04, 2);
            int Rrange = Rrangesensorcache[0] & 0xFF;

            Lrangesensorcache = Lrangereader.read(0x04, 2);
            int Lrange = Lrangesensorcache[0] & 0xFF;
            telemetry.addData("ultrar", ultraR.getUltrasonicLevel());
            telemetry.addData("ultral", ultraL.getUltrasonicLevel());
            telemetry.addData("shootertouch", shootertouch.isPressed());
            telemetry.addData("Rultrasonic", Rrange);
            telemetry.addData("Lultrasonic", Lrange);
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.addData("Flight ", Flight.getRawLightDetected());
            telemetry.addData("Blight ", Blight.getRawLightDetected());
            telemetry.addData("Lcolor blue", Lcolor.blue());
            telemetry.addData("Lcolor red", Lcolor.red());
            telemetry.addData("Rcolor blue", Rcolor.blue());
            telemetry.addData("Rcolor red", Rcolor.red());
            */


            double Xacc;
            Xacc = imu.getLinearAcceleration().xAccel;
            double Yacc;
            Yacc = imu.getLinearAcceleration().yAccel;

            double postX;
            postX = imu.getPosition().x;
            double postY;
            postY = imu.getPosition().y;


            //telemetry.addData("LinearAcc: ", imu.getLinearAcceleration());
            //telemetry.addData("angular velocity", imu.getAngularVelocity());

            telemetry.addData("Xacc: ", Math.round(Xacc));
            telemetry.addData("Yacc: ", Math.round(Yacc));
            telemetry.addData("PosX: ", postX);
            telemetry.addData("PosY: ", postY);
            telemetry.update();
            idle();
        }


    }
}
