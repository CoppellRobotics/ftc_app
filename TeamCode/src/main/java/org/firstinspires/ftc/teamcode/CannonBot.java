package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by oxg0829 on 12/5/2016.
 * Editied by Marsfan (gabe)
 * this is the main code for our teleop mode.
 */


@TeleOp(name="2017 Teleop main")
public class CannonBot extends OpMode {

    private JoyStick js = new JoyStick();
    private DcMotor sweeper;
    private DcMotor launcher;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo pusher;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        launcher = hardwareMap.dcMotor.get("launcher");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        pusher = hardwareMap.servo.get("buttonPusher");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        js.init(gamepad1);

    }
    @Override
    public void loop() {
        if(js.leftTrigger()){
            sweeper.setPower(1);
        }else if (js.rightTrigger()){
            sweeper.setPower(-1);
        }else{
            sweeper.setPower(0);
        }
        if(js.dpadLeft()){
            pusher.setPosition(0);
        }else if(js.dpadRight()){
            pusher.setPosition(1);
        }
        leftMotor.setPower(js.leftY(true));
        rightMotor.setPower(js.rightY(true));
        telemetry.addData("LeftY", js.leftY(true));
        telemetry.addData("rightY", js.rightY(true));
        telemetry.update();
    }


}