package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="BasicTankDrive2M ", group="FTC")
@Disabled
public class BasicTankDrive2M extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double Rpower;
        double Lpower;

        double fastency = 1;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");

        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);


        waitOneFullHardwareCycle();
        waitForStart();
        while (opModeIsActive()) {
            waitOneFullHardwareCycle();

            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.4;


            Lpower = gamepad1.left_stick_y;
            Rpower = gamepad1.right_stick_y;

            LFdrive.setPower(Lpower*fastency);
            LBdrive.setPower(Lpower*fastency);
            RFdrive.setPower(Rpower*fastency);
            RBdrive.setPower(Rpower*fastency);

        }
    }
}
