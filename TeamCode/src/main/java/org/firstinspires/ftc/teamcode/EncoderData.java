package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.Timer;
import java.util.TimerTask;

import org.firstinspires.ftc.teamcode.JoyStick;


/**
 * Created by Gabe on 2/6/2016.
 */
@TeleOp(name="Encoder Value Read")
public class EncoderData extends OpMode {
    //declare variables that will represent motors to make life eaiser
    DcMotor leftMotor;
    DcMotor rightMotor;
    double leftEnc;
    double rightEnc;
    JoyStick js = new JoyStick();

    @Override
    public void init() {
        js.init(gamepad1);
        leftMotor = hardwareMap.dcMotor.get("leftMotor"); //get references to the motors from the hardware map
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//motors run without encoder
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse the right motor
        //  leftMotor.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void loop() {
        leftMotor.setPower(js.leftY());
        rightMotor.setPower(js.rightY());
        leftEnc = (leftMotor.getCurrentPosition() / (double) -1440 * 360); //degrees we have rotated
        leftEnc = Math.toRadians(leftEnc);
        leftEnc = leftEnc * 5;
        rightEnc = (rightMotor.getCurrentPosition() / (double) -1440 * 360); //degrees we have rotated
        rightEnc =Math.toRadians(rightEnc);
        rightEnc = rightEnc * 5;
        //TODO add conversion to distance travled (just to test)
        telemetry.addData("LeftMotor Pwr", leftMotor.getPower());
        telemetry.addData("RightMotor Pwr", rightMotor.getPower());
        telemetry.addData("LeftMotor Pos", leftEnc + "cm");
        telemetry.addData("RightMotor Pos", rightEnc + "cm");
        telemetry.update();
    }

}
