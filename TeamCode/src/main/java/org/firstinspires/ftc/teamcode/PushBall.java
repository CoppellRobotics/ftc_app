package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Gabe on 1/21/2017.
 */

@Autonomous(name = "Ball Crusher")
public class PushBall extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    @Override
    public void runOpMode() throws InterruptedException {


        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
       // leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

            leftMotor.setTargetPosition(-(1440 * 4));
           // leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setPower(1);
            rightMotor.setPower(1);
            while (leftMotor.isBusy() && leftMotor.getCurrentPosition() > (1440*4)) {
                telemetry.addData("LENC", leftMotor.getCurrentPosition());
                telemetry.addData("RENC", rightMotor.getCurrentPosition());
                telemetry.update();
                idle();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);

    }
}
