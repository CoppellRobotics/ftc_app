package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Gabe on 12/15/2016.
 */


@Autonomous(name ="blueBot")
public class VisionBotAutomBlue extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    enum State {findingTarget, turning, aligning, backturning, analysis, positioning, pressing, done};
    State state;

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQ92H9H/////AAAAGTitPu+5QUlxl/5DeeMeZe9kUysq4fNXHaSrlNBKmasiCDfkzw+g8z6R+f1SZeDvSXrJd7JwHLedujT8NsMHAr8PRfGX011IMcYomFzn9VwS8MyaUXNeMaUzY7NPEC9cLzg0dJrxPWj101l09+K3d1bKa3jEc1271jRgAwzAnI80Eh0g0mK/8mCMW9zdXLjTH1xJ9T7qtTMUN3DQVo2FY3u+askvEVGFashI+6mZtFk4SAgoy2XY1fYqXiZN1Wz1gVCqyF8Hxi9KuoX/awJz+SI/jdgQn2nmp+aHgw1Hcm9oXL5ZB4UMFD7zV94Bg2sLbanoN6h3dTtIpYXGZgDzPWGMgDWisjJV3TvFTTVauFIK";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheels = targets.get(0);
        wheels.setName("Wheels");
        VuforiaTrackable tools = targets.get(1);
        tools.setName("Tools");
        VuforiaTrackable legos = targets.get(2);
        legos.setName("Legos");
        VuforiaTrackable gears = targets.get(3);
        gears.setName("Gears");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmPerFoot        = 304.8f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


        OpenGLMatrix wheelLoc = OpenGLMatrix
                .translation(mmPerFoot * 1, -mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 90));
        wheels.setLocation(wheelLoc);
        RobotLog.ii(TAG, "Wheels Location=%s", format(wheelLoc));


        OpenGLMatrix toolLoc = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, mmPerFoot * 3, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolLoc);
        RobotLog.ii(TAG, "Tools Location=%s", format(toolLoc));




        OpenGLMatrix legoLoc = OpenGLMatrix
                .translation(-mmPerFoot*3, -mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 90));
        legos.setLocation(legoLoc);
        RobotLog.ii(TAG, "Legos Location=%s", format(legoLoc));



        OpenGLMatrix gearLoc = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, -mmPerFoot *1, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearLoc);
        RobotLog.ii(TAG, "Gear Location=%s", format(gearLoc));


        OpenGLMatrix phoneLoc = OpenGLMatrix
                .translation(0,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, 0
                ));
        RobotLog.ii(TAG, "Phone Location=%s", format(phoneLoc));

        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLoc, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLoc, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLoc, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLoc, parameters.cameraDirection);


        telemetry.addData("str", "str");
        waitForStart();
        state = State.findingTarget;

        targets.activate();


        while (opModeIsActive()) {

                for (VuforiaTrackable trackable : allTrackables) {
                    telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }

            switch(state){
                case findingTarget:
                    if(lastLocation == null){
                        leftMotor.setPower(.5);
                        rightMotor.setPower(.5);
                    }else{
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = state.turning;

                    }
                    telemetry.addData("state", "finding");
                    break;

                case turning:
                    if (getOrientation(lastLocation).thirdAngle < 85) {
                        rightMotor.setPower(.5);
                        leftMotor.setPower(-.5);
                    } else if (getOrientation(lastLocation).thirdAngle > 95) {
                        leftMotor.setPower(.5);
                        rightMotor.setPower(-.5);
                    } else {
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = State.aligning;
                  }
                    telemetry.addData("state", "turning");
                    break;

                case aligning:
                    if(getTranslation(lastLocation).get(0) > 304){
                        leftMotor.setPower(.5);
                        rightMotor.setPower(.5);
                    }else{
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = State.backturning;
                    }
                    telemetry.addData("state", "aligning");

                    break;
                case backturning:
                    if(getOrientation(lastLocation).firstAngle > 0){
                        leftMotor.setPower(.5);
                        rightMotor.setPower(-.5);
                    }else{
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = state.done;
                    }
                    state = state.positioning;
                    telemetry.addData("state", "backturning");
                    break;
                case analysis:
                    state = state.positioning;
                    telemetry.addData("state", "analysis");
                    break;
                case positioning:
                    state = state.pressing;
                    telemetry.addData("state", "positioning");
                    break;
                case pressing:
                    state = state.done;
                    telemetry.addData("state", "pressing");
                    break;
                case done:
                    telemetry.addData("state", "done!");
                    break;
            }
            telemetry.update();
            if(lastLocation != null){
                telemetry.addData("Pos", format(lastLocation));
            }else{
                telemetry.addData("Pos", "null");
            }






        }

}

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    Orientation getOrientation(OpenGLMatrix transformationMatrix){
        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return orientation;
    }
    VectorF getTranslation(OpenGLMatrix transformationMatrix){
        VectorF translation = transformationMatrix.getTranslation();
        return  translation;
    }
}
