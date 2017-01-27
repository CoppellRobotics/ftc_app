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
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        leftMotor.setTargetPosition(1440 * 4);
        rightMotor.setTargetPosition(1440 * 4);
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        while(leftMotor.isBusy() || rightMotor.isBusy()){
            idle();
        }
    }
}
