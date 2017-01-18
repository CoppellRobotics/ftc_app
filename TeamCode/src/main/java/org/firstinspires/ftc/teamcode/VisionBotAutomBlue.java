package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

/**
 * Created by Gabe on 12/15/2016.
 */

@Autonomous(name="VisionBotBlue")
public class VisionBotAutomBlue extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;


    @Override
    public void runOpMode() throws InterruptedException {

        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;


        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
       /* leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AQ92H9H/////AAAAGTitPu+5QUlxl/5DeeMeZe9kUysq4fNXHaSrlNBKmasiCDfkzw+g8z6R+f1SZeDvSXrJd7JwHLedujT8NsMHAr8PRfGX011IMcYomFzn9VwS8MyaUXNeMaUzY7NPEC9cLzg0dJrxPWj101l09+K3d1bKa3jEc1271jRgAwzAnI80Eh0g0mK/8mCMW9zdXLjTH1xJ9T7qtTMUN3DQVo2FY3u+askvEVGFashI+6mZtFk4SAgoy2XY1fYqXiZN1Wz1gVCqyF8Hxi9KuoX/awJz+SI/jdgQn2nmp+aHgw1Hcm9oXL5ZB4UMFD7zV94Bg2sLbanoN6h3dTtIpYXGZgDzPWGMgDWisjJV3TvFTTVauFIK";
        params.useExtendedTracking = true;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
        VuforiaTrackableDefaultListener tools = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();
        VuforiaTrackableDefaultListener lego = (VuforiaTrackableDefaultListener) beacons.get(2).getListener();
        VuforiaTrackableDefaultListener gears = (VuforiaTrackableDefaultListener) beacons.get(3).getListener();


        waitForStart();
        beacons.activate();


        leftMotor.setPower(1);
        rightMotor.setPower(1);
        while (opModeIsActive()) {
            while (opModeIsActive() && tools.getRawPose() == null) {
                idle();
            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            OpenGLMatrix toolsPose = tools.getPose();
            OpenGLMatrix legoPose = lego.getPose();
            OpenGLMatrix gearsPose = gears.getPose();
            if (toolsPose != null) {
                double ZDist = toolsPose.getTranslation().get(2);
                ZDist = .056328* ZDist + 8.2697;
                VectorF angles = anglesFromTarget(tools);
                VectorF trans = navOffWall(toolsPose.getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
                telemetry.addData("x", toolsPose.getTranslation().get(0));
                telemetry.addData("y", toolsPose.getTranslation().get(1));
                telemetry.addData("z", ZDist);
                double dist = Math.sqrt((toolsPose.getTranslation().get(0) * toolsPose.getTranslation().get(0)) + (ZDist * ZDist));
                telemetry.addData("dist:", dist);
                telemetry.addData("rt", Math.sqrt(4));
                //telemetry.addData("rotation", Math.toDegrees(Math.atan2(toolsPose.getTranslation().get(2), toolsPose.getTranslation().get(0)))+90);
                // if(toolsPose.getTranslation().get(0) > 0){
                telemetry.addData("rotation", (Math.atan(toolsPose.getTranslation().get(2) / toolsPose.getTranslation().get(0)))*(180/ Math.PI));
          //  }
            // telemetry.addData("angles", angles);
        }

        telemetry.update();


    }

}
    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) *
                Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) *
                Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) *
                Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}
