package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by oxg0829 on 11/3/2016.
 */

public class OperatingArm extends OpMode {
    DcMotor leftArm;

    public void init(){

                leftArm = hardwareMap.dcMotor.get("left_arm");
    }
}

public void loop() {

    if (gamepad1.y) {
        leftArm.setPower(0.1);
    } else {
        leftArm.setPower(0);
    }
    if (gamepad1.b) leftArm.setPower(-0.1);
}