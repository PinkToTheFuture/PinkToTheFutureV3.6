package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="IntakeConcept", group="FTC")
public class IntakeConcept extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double Rpower;
        double Lpower;

        double speed = 1;

        DcMotor Ldrive  = hardwareMap.dcMotor.get("Lintake");
        DcMotor Rdrive = hardwareMap.dcMotor.get("Rintake");

        Rdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitOneFullHardwareCycle();
        waitForStart();
        while (opModeIsActive()) {
            waitOneFullHardwareCycle();

            if (gamepad1.dpad_up)     speed = 1;
            if (gamepad1.dpad_down)   speed = 0.4;

            Rpower =0;
            Lpower=0;


            Lpower = gamepad1.left_trigger;
            Rpower = gamepad1.right_trigger;

            Ldrive.setPower(Lpower*speed);
            Rdrive.setPower(Rpower*speed);

        }
    }
}