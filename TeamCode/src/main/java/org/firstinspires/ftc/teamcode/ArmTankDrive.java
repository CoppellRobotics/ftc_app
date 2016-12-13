package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Created by oxg0829 on 11/14/2016.
 */

public class ArmTankDrive extends OpMode {
    DcMotor rightMotor;
    DcMotor leftMotor;
    public DcMotor leftArm;
    public Object hardwareMap;


    @Override
    public void init() {
        leftMotor = hardwareMap.DcMotor.get("left_motor");
        rightMotor = hardwareMap.DcMotor.get("right_motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }


    @Override
     void loop(Gamepad gamepad1) {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.left_stick_y;

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);
        if (gamepad1.y) {
            leftArm.setPower(0.1);
        } else {
            leftArm.setPower(0.0);
        }
        if (gamepad.b){
            setPower(0.0);
        }
    }

}