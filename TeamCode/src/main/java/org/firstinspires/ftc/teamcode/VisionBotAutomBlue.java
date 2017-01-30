package org.firstinspires.ftc.teamcode;



import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
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
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;



/**
 * Created by Gabe R. on 12/15/2016.
 * This should drive to the target, scan it, and push the correct button
 */


@Autonomous(name ="blueBot")
public class VisionBotAutomBlue extends LinearOpMode {

    private final static Scalar blueLow = new Scalar(108, 0, 220);
    private final static Scalar blueHigh = new Scalar(178, 255, 255);

    private int BEACON_NOT_VISIBLE = 0;
    private int BEACON_RED_BLUE = 1;
    private int BEACON_BLUE_RED = 2;
    private int BEACON_ALL_BLUE = 3;
    private int BEACON_NO_BLUE = 4;


    private DcMotor leftMotor;
    private DcMotor rightMotor;


    enum State {findingTarget, turning, aligning, backturning, analysis, positioning, pressing, done};
    private State state;

    private static final String TAG = "Vuforia Sample";

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQ92H9H/////AAAAGTitPu+5QUlxl/5DeeMeZe9kUysq4fNXHaSrlNBKmasiCDfkzw+g8z6R+f1SZeDvSXrJd7JwHLedujT8NsMHAr8PRfGX011IMcYomFzn9VwS8MyaUXNeMaUzY7NPEC9cLzg0dJrxPWj101l09+K3d1bKa3jEc1271jRgAwzAnI80Eh0g0mK/8mCMW9zdXLjTH1xJ9T7qtTMUN3DQVo2FY3u+askvEVGFashI+6mZtFk4SAgoy2XY1fYqXiZN1Wz1gVCqyF8Hxi9KuoX/awJz+SI/jdgQn2nmp+aHgw1Hcm9oXL5ZB4UMFD7zV94Bg2sLbanoN6h3dTtIpYXGZgDzPWGMgDWisjJV3TvFTTVauFIK";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);


        VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("FTC_2016-17");
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


        OpenGLMatrix wheelLoc = OpenGLMatrix   //TODO will need to move axes to fix for height, possibly xy corrdintes as well
                .translation(mmPerFoot * 1, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelLoc);
        RobotLog.ii(TAG, "Wheels Location=%s", format(wheelLoc));


        OpenGLMatrix toolLoc = OpenGLMatrix  //TODO will need to move axes to fix for height, possibly xy corrdintes as well
                .translation(-mmFTCFieldWidth/2, mmPerFoot * 3, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolLoc);
        RobotLog.ii(TAG, "Tools Location=%s", format(toolLoc));




        OpenGLMatrix legoLoc = OpenGLMatrix  //TODO will need to move axes to fix for height, possibly xy corrdintes as well
                .translation(-mmPerFoot*3, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legoLoc);
        RobotLog.ii(TAG, "Legos Location=%s", format(legoLoc));



        OpenGLMatrix gearLoc = OpenGLMatrix   //TODO will need to move axes to fix for height, possibly xy corrdintes as well
                .translation(-mmFTCFieldWidth/2, -mmPerFoot *1, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearLoc);
        RobotLog.ii(TAG, "Gear Location=%s", format(gearLoc));


        OpenGLMatrix phoneLoc = OpenGLMatrix    //TODO will need to calibrate the phone location.
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
        VuforiaTrackableDefaultListener wheelsL = (VuforiaTrackableDefaultListener) targets.get(0).getListener();


        telemetry.addData("str", "str");
        waitForStart();
        state = State.findingTarget;

        targets.activate();


        while (opModeIsActive()) {

                for (VuforiaTrackable trackable : allTrackables) {
                    telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }

            switch(state){
                case findingTarget:
                    if(lastLocation == null){
                        leftMotor.setPower(.25);
                        rightMotor.setPower(.25);
                    }else{
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = State.turning;

                    }
                    telemetry.addData("state", "finding");
                    break;

                case turning:
                    if (getOrientation(lastLocation).thirdAngle < 85) {
                        rightMotor.setPower(.25);
                        leftMotor.setPower(-.25);
                    } else if (getOrientation(lastLocation).thirdAngle > 95) {
                        leftMotor.setPower(.25);
                        rightMotor.setPower(-.25);
                    } else {
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = State.aligning;
                  }
                    telemetry.addData("state", "turning");
                    break;

                case aligning:
                    if(getTranslation(lastLocation).get(0) > 310){
                        leftMotor.setPower(.25);
                        rightMotor.setPower(.25);
                    }else{
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = State.backturning;
                    }
                    telemetry.addData("state", "aligning");

                    break;
                case backturning:
                    if(getOrientation(lastLocation).thirdAngle > 0){
                        leftMotor.setPower(.25);
                        rightMotor.setPower(-.25);
                    }else{
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = State.analysis;
                    }
                    telemetry.addData("state", "backturning");
                    break;
                case analysis:
                    int config = getBeaconConfig(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), wheelsL, vuforia.getCameraCalibration());

                    telemetry.addData("config", config);

                    /* if (config == BEACON_BLUE_RED){
                        telemetry.addData("config", "BLUERED");
                    }else if (config == BEACON_RED_BLUE){
                        telemetry.addData("config", "REDBLUE");
                    }else{
                        telemetry.addData("config", "ERROR");
                    }*/

                    telemetry.update();
                    state = State.backturning;
                    telemetry.addData("state", "analysis");

                    break;
                case positioning:
                    telemetry.addData("state", "positioning");
                    break;
                case pressing:
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

    public int test(){
        for(int i = 0; i < 10; i++){
            if(i == 5){
                return 1;
            }
        }
        return 2;
    }

    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    private Orientation getOrientation(OpenGLMatrix transformationMatrix){
        return Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    private VectorF getTranslation(OpenGLMatrix transformationMatrix){
        return transformationMatrix.getTranslation();
    }

    @Nullable
    private Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for
        return null;

    }

    private int getBeaconConfig(Image img, VuforiaTrackableDefaultListener  beacon, CameraCalibration camCal){

        OpenGLMatrix pose = beacon.getRawPose();

        if(pose != null && img != null && img.getPixels() != null){

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left //TODO moves cropping system up. will require calibration
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData();  //upper right //TODO moves cropping system up. will require calibration
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, -92, 0)).getData();  //bottom right //TODO moves cropping system up. will require calibration
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, -92, 0)).getData();  //bottom left //TODO moves cropping system up. will require calibration

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            Mat mask = new Mat();
            Core.inRange(cropped, blueLow, blueHigh, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            Log.i("CentroidX", "" + ((mmnts.get_m10() / mmnts.get_m00())));
            Log.i("CentroidY", "" + ((mmnts.get_m01() / mmnts.get_m00())));


            if(mmnts.get_m00() > mask.total() * .8){
                telemetry.addData("status", "all blue");
                return BEACON_ALL_BLUE;
            }else if (mmnts.get_m00() < mask.total()* .1){
                telemetry.addData("status", "no blue");
                return BEACON_NO_BLUE;

            }

            if((mmnts.get_m01() / mmnts.get_m00() < cropped.rows() / 2)){
                return BEACON_RED_BLUE;
            }else {
                return BEACON_BLUE_RED;
            }


        }

        return BEACON_NOT_VISIBLE;

    }



}
