package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="BasicTankDrive", group="FTC")
public class BasicTankDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double Rpower;
        double Lpower;

        double fastency = 1;

        DcMotor Ldrive  = hardwareMap.dcMotor.get("Ldrive");
        DcMotor Rdrive = hardwareMap.dcMotor.get("Rdrive");

        Rdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitOneFullHardwareCycle();
        waitForStart();
        while (opModeIsActive()) {
            waitOneFullHardwareCycle();

            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.4;


            Lpower = gamepad1.left_stick_y;
            Rpower = gamepad1.right_stick_y;

            Ldrive.setPower(Lpower*fastency);
            Rdrive.setPower(Rpower*fastency);

        }
    }
}
