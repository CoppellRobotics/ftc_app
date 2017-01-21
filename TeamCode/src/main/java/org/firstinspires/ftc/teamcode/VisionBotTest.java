package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.JoyStick;

/**
 * Created by Gabe R. on 10/24/2016.
 */


@TeleOp(name="VisionBot")
public class VisionBotTest extends OpMode{
    JoyStick js = new JoyStick();
    DcMotor leftMotor;
    DcMotor rightMotor;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        js.init(gamepad1);
    }

    @Override
    public void loop() {
        leftMotor.setPower(js.leftY());
        rightMotor.setPower(js.rightY());
    }


}
