package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.jar.Attributes;

/**
 * Created by Gabe on 11/17/2016.
 * This file simply does nothing. It will allow for us to check for some basic errors
 */

@TeleOp(name="Nullop")
public class NullOp extends OpMode{
    @Override
    public void init() {
     DcMotor leftMotor = hardwareMap.dcMotor.get("leftMotor");
    }

    @Override
    public void loop() {

    }
}
