package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by oxg0829 on 11/3/2016.
 */

public class TankDrive extends OpMode{



DcMotor leftMotor;
        DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.left_stick_y;

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

    }
}

