package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.robotUtils;

/**
 * Created by Gabe on 12/17/2016.
 */

@TeleOp(name="drive 40cm")
public class Autom40 extends LinearOpMode{
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private double leftEnc;
    private double rightEnc;
    int val1;
    int val2;

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
        leftMotor.setTargetPosition(robot.inchesToTicks(16));
        rightMotor.setTargetPosition(robot.inchesToTicks(16));
        leftMotor.setPower(.15);
        rightMotor.setPower(.15);

        while (leftMotor.isBusy() && rightMotor.isBusy()){
            telemetry.addData("LeftM", leftMotor.getCurrentPosition());
            telemetry.addData("rightM", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
