package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Gabe on 10/31/2016.
 */

public class JoyStick{

        private float leftY;
        private float rightY;
        private Gamepad gamepad1;


public void init(Gamepad gp){
    gamepad1 = gp;
}

    public float leftY(boolean squared){
            if(squared) {
                    float leftYAxis = gamepad1.left_stick_y;
                    if (leftYAxis < 0) {
                            leftYAxis = leftYAxis * leftYAxis;
                    } else {
                            leftYAxis = leftYAxis * leftYAxis * -1;
                    }
                    return leftYAxis;
            }
            else{
                return -gamepad1.left_stick_y;
            }
            }

    public float leftY(){
        float leftYAxis = gamepad1.left_stick_y;
        if (leftYAxis < 0) {
            leftYAxis = leftYAxis * leftYAxis;
        } else {
            leftYAxis = leftYAxis * leftYAxis * -1;
        }
        return leftYAxis;
    }


    public float rightY(boolean squared) {
        if (squared) {
            float rightYAxis = gamepad1.right_stick_y;
            if (rightYAxis < 0) {
                rightYAxis = rightYAxis * rightYAxis;
            } else {
                rightYAxis = rightYAxis * rightYAxis * -1;
            }
            return rightYAxis;
        } else{
            return -gamepad1.right_stick_y;
        }
    }

    public float rightY(){
        float rightYAxis = gamepad1.right_stick_y;
        if (rightYAxis < 0) {
            rightYAxis = rightYAxis * rightYAxis ;
        } else {
            rightYAxis = rightYAxis * rightYAxis * -1;
        }
        return rightYAxis;
    }

    public float rightX(boolean squared){
        if(squared){
            float rightXAxis = gamepad1.right_stick_x;
            if(rightXAxis < 0){
                rightXAxis = rightXAxis * rightXAxis * -1;
            }else{
                rightXAxis = rightXAxis * rightXAxis;
            }
            return rightXAxis;
        }else{
            return gamepad1.right_stick_x;
        }
    }

    public float rightX(){
        float rightXAxis = gamepad1.right_stick_x;
        if(rightXAxis < 0){
            rightXAxis = rightXAxis * rightXAxis * -1;
        }else{
            rightXAxis = rightXAxis * rightXAxis;
        }
        return rightXAxis;
    }

    public float leftX(boolean squared){
        if(squared){
            float leftXAxis = gamepad1.left_stick_x;
            if(leftXAxis < 0){
                leftXAxis = leftXAxis * leftXAxis * -1;
            }else{
                leftXAxis = leftXAxis * leftXAxis;
            }
            return leftXAxis;
        }else{
            return gamepad1.right_stick_x;
        }
    }

    public float lefX(){
        float leftXAxis = gamepad1.left_stick_x;
        if(leftXAxis < 0){
            leftXAxis = leftXAxis * leftXAxis * -1;
        }else{
            leftXAxis = leftXAxis * leftXAxis;
        }
        return leftXAxis;
    }

    public boolean leftButton(){
        return gamepad1.left_stick_button;
    }

    public boolean rightButton(){
        return gamepad1.right_stick_button;
    }

    public boolean start(){
        return gamepad1.start;
    }

    public boolean back(){
        return  gamepad1.back;
    }

    public boolean dpadUp(){
        return gamepad1.dpad_up;
    }

    public boolean dpadDown(){
        return gamepad1.dpad_down;
    }

    public boolean dpadLeft(){
        return gamepad1.dpad_left;
    }

    public boolean dpadRight(){
        return gamepad1.dpad_right;
    }

    public boolean x(){
        return gamepad1.x;
    }

    public boolean y(){
        return gamepad1.y;
    }

    public boolean a(){
        return gamepad1.a;
    }

    public boolean b(){
        return gamepad1.b;
    }

    public boolean leftTrigger(){
        return gamepad1.left_bumper;
    }

    public boolean rightTrigger(){
        return gamepad1.right_bumper;
    }

}