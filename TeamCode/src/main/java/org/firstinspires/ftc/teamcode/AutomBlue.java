package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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

import java.util.Arrays;

/**
 * Created by Gabe on 1/24/2017.
 */


@Autonomous(name = "testing")
public class AutomBlue extends LinearOpMode {


    public final static int NOT_VISIBLE = 0;

    public final static int BEACON_BLUE_RED = 1;
    public final static int BEACON_RED_BLUE = 2;
    public final static int BEACON_ALL_BLUE = 3;

    public final static Scalar BEACON_BLUE_LOW = new Scalar(108, 0, 220);
    public final static Scalar BEACON_BLUE_HIGH = new Scalar(178, 255, 255);

    @Override
    public void runOpMode() throws InterruptedException {

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "Ad0I0ir/////AAAAAfR3NIO1HkxSqM8NPhlEftFXtFAm6DC5w4Cjcy30" +
                "WUdGozklFlAkxeHpjfWc4moeL2ZTPvZ+wAoyOnlZxyB6Wr1BRE9154j6K" +
                "1/8tPvu21y5ke1MIbyoJ/5BAQuiwoAadjptZ8fpS7A0QGPrMe0VauJIM1" +
                "mW3UU2ezYFSOcPghCOCvQ8zid1Bb8A92IkbLcBUcv3DEC6ia4SEkbRMY7" +
                "TpOh2gzsXdsue4tqj9g7vj7zBU5Hu4WhkMDJRsThn+5QoHXqvavDsCElw" +
                "mDHG3hlEYo7qN/vV9VcQUX9XnVLuDeZhkp885BHK5vAe8T9W3Vxj2H/R4" +
                "oijQso6hEBaXsOpCHIWGcuphpoe9yoQlmNRRZ97";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        locale.setFrameQueueCapacity(1);

        VuforiaTrackables beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackableDefaultListener gears = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        waitForStart();
        beacons.activate();

        while (!gears.isVisible()) {
            delay(1);
        }//while

        while (opModeIsActive()) {
            int beaconConfig = NOT_VISIBLE;
            while (beaconConfig == NOT_VISIBLE) {
                beaconConfig = getBeaconConfig(getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565), gears, locale.getCameraCalibration());
            }//while

            if (beaconConfig == BEACON_RED_BLUE) {
                Log.i("RED", "BLUE");
            } else if (beaconConfig != NOT_VISIBLE) {
                Log.i("BLUE", "RED");
            } else {
                Log.i("BEAC", "== -1");
            }//else

            delay(500);
        }
    }

    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }


    @Nullable
    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }
        }

        return null;
    }



    public static int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        OpenGLMatrix pose = beacon.getRawPose();

        if (pose != null && img != null && img.getPixels() != null) {

            Mat crop = new Mat(2, 2, CvType.CV_8UC3);

            /*Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

            rawPose.setData(poseData);

            //calculating pixel coordinates of beacon corners
            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, -92, 0)).getData(); //lower right of beacon
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, -92, 0)).getData(); //lower left of beacon

            //getting camera image...
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_64F);


            //turning the corner pixel coordinates into a proper bounding box
            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));


            //make sure our bounding box doesn't go outside of the image
            //OpenCV doesn't like that...
            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            //cropping bounding box out of camera image
            final Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            //filtering out non-beacon-blue colours in HSV colour space
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            //get filtered mask
            //if pixel is within acceptable blue-beacon-colour range, it's changed to white.
            //Otherwise, it's turned to black
            Mat mask = new Mat();

            Core.inRange(cropped, BEACON_BLUE_LOW, BEACON_BLUE_HIGH, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            //calculating centroid of the resulting binary mask via image moments
            Log.i("CentroidX", "" + ((mmnts.get_m10() / mmnts.get_m00())));
            Log.i("CentroidY", "" + ((mmnts.get_m01() / mmnts.get_m00())));

            //checking if blue either takes up the majority of the image (which means the beacon is all blue)
            //or if there's barely any blue in the image (which means the beacon is all red or off)
//            if (mmnts.get_m00() / mask.total() > 0.8) {
//                return VortexUtils.BEACON_ALL_BLUE;
//            } else if (mmnts.get_m00() / mask.total() < 0.1) {
//                return VortexUtils.BEACON_NO_BLUE;
//            }//elseif

            //Note: for some reason, we end up with a image that is rotated 90 degrees
            //if centroid is in the bottom half of the image, the blue beacon is on the left
            //if the centroid is in the top half, the blue beacon is on the right
            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2) {
                return BEACON_RED_BLUE;
            } else {
                return BEACON_BLUE_RED;
            }//else*/
        }//if*;

        return NOT_VISIBLE;

    }//getBeaconConfig

    public static void delay(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
            e.printStackTrace();
        }//catch
    }//wait

  /*  public static Mat bitmapToMat (Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }*/
}
