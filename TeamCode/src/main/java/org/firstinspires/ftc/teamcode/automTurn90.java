package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Gabe on 12/18/2016.
 */

public class automTurn90 extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private double leftEnc;
    private double rightEnc;

    robotUtils robot = new robotUtils(13, 4);


    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //get references to the motors from the hardware map
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMaxSpeed(3300);
        rightMotor.setMaxSpeed(3300);
        waitForStart();

        leftMotor.setTargetPosition(robot.degreesToTicksSingleMotor(90));
        leftMotor.setPower(.1);

        while(leftMotor.isBusy() || rightMotor.isBusy()){
            idle();
        }
    }
}
