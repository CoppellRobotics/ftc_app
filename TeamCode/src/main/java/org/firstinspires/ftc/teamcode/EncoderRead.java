package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


/**
 * Created by oxg0829 on 10/27/2016.
 */

public class EncoderRead extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;

    String rightMotorName;
    String leftMotorName;


    leftMotor = hardwareMap.dcMotor.(leftMotorName);
    rightMotor = hardwareMap.dcMotor.(rightMotorName);
}
