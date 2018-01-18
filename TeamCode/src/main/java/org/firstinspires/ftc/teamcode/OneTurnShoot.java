package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@Autonomous(name = "reboot shooter", group = "corner")

public class OneTurnShoot extends LinearOpMode implements org.firstinspires.ftc.teamcode.ServoVariables {




    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Servo Armservo = hardwareMap.servo.get("servoarm");
        Armservo.setPosition(ArmservoStopPosition);
        Servo releaseArmL = hardwareMap.servo.get("releasearmL");
        Servo releaseArmR = hardwareMap.servo.get("releasearmR");
        releaseArmL.setPosition(releaseArmLStartPosition);
        releaseArmR.setPosition(releaseArmRStartPosition);
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");
        shooterservoX.setPosition(shooterservoXStartPosition);
        TouchSensor shootertouch = hardwareMap.touchSensor.get("shootertouch");

        shooterservoX.setPosition(0.13);
        sleep(400);
        while (opModeIsActive() && !shootertouch.isPressed()){
            idle();
        }
        shooterservoX.setPosition(0.5);


        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                shooterservoX.setPosition(0.4);
            } else {
                shooterservoX.setPosition(0.5);
            }
        }




    }

}
