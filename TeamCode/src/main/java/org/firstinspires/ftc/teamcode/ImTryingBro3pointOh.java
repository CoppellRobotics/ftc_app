package org.firstinspires.ftc.teamcode;

/**
 * Created by eah1759 on 10/27/2016.
 */

public class ImTryingBro3pointOh
{
    public ImTryingBro3pointOh()
    {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMoto = hardwareMap.dcMotor.get("rightMoto");

        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad1.right_trigger;
        double roY = rightY - 0.5;
        double rY = roY * 2;
        double left2 = leftY * leftY;
        double right2 = rY * rY;

        if(leftY < 0) {
            leftMotor.setPower(-left2);
        } else {
            leftMotor.setPower(left2);
        }
        if(rY < 0) {
            rightMoto.setPower(right2);
        } else {
            rightMoto.setPower(-right2);
        }
        telemetry.addData("LeftMotor", leftMotor.getPower());
        telemetry.addData("RightMotor", rightMoto.getPower());
    }
}
