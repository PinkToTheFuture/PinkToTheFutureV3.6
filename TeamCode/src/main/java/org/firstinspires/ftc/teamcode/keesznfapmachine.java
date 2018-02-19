package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="BasickeesFAP", group="FTC")
public class keesznfapmachine extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int Gpos1 = 0;
        int Gpos2 = 1000;
        int hoi = 0;
        double times = getRuntime();
        boolean loop = false;
        boolean pos = true;

        DcMotor geleider = hardwareMap.dcMotor.get("geleider");
        geleider.setDirection(DcMotorSimple.Direction.REVERSE);
        geleider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        geleider.setPower(1);
        waitOneFullHardwareCycle();
        waitForStart();

        while (opModeIsActive()) {
            waitOneFullHardwareCycle();

            if (gamepad1.dpad_up)     Gpos1 = geleider.getCurrentPosition();
            if (gamepad1.dpad_down)   Gpos2 = geleider.getCurrentPosition();
            hoi = (int) (hoi+(gamepad1.left_stick_y*20));
            if (gamepad1.a)   loop = true;
            geleider.setTargetPosition(hoi);


            while (loop && opModeIsActive()){
                if (times < getRuntime()){
                    if (pos){
                        geleider.setTargetPosition(Gpos1);
                        pos = false;
                        times = times + 3;
                    }
                }
                if (times < getRuntime()){
                    if (!pos){
                        geleider.setTargetPosition(Gpos2);
                        pos = true;
                        times = times + 3;
                    }

                }

                if (gamepad1.b)  loop = false;

            }

            telemetry.addData("motor: ", geleider.getCurrentPosition());
            telemetry.addData("gpos1", Gpos1);
            telemetry.addData("gpos2", Gpos2);
            telemetry.addData("time: ", times);
            telemetry.update();
        }
    }
}
