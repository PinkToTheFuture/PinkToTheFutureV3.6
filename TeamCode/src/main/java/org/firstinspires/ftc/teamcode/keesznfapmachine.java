package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="BasicmokkyFAP", group="FTC")
public class keesznfapmachine extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        double hoi = 0;
        Servo bakjeturn = hardwareMap.servo.get("bakjeturn");

        waitOneFullHardwareCycle();
        waitForStart();

        while (opModeIsActive()) {
            waitOneFullHardwareCycle();

            if (gamepad1.a) hoi = hoi + 0.008;
            if (gamepad1.b) hoi = hoi - 0.008;
            bakjeturn.setPosition(hoi);
        }
    }
}
