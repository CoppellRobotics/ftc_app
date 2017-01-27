package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.objects.Ellipse;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

/**
 * Created by Gabe on 1/21/2017.
 */

@Autonomous(name = "test")
public class EncoderImage extends LinearVisionOpMode{

    private DcMotor leftMotor;
    private DcMotor rightMotor;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForVisionStart();                       //wait for camera to init

        this.setCamera(Cameras.PRIMARY);            //set camera. Primary is the big one
        this.setFrameSize(new Size(900, 900));      //set camera view dimensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX); //set analysis type, we currently are using fast, but the others should be better

        beacon.setColorToleranceBlue(0);
        beacon.setColorToleranceRed(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();


        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        leftMotor.setTargetPosition(1440*3);
        rightMotor.setTargetPosition(1440*3);
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);

        while(leftMotor.isBusy() || leftMotor.isBusy()){
        }

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rotate 90 degrees

        boolean state = beacon.getAnalysis().isLeftBlue();

            Point alignment = beacon.getAnalysis().getCenter();


            telemetry.addData("location", beacon.getAnalysis().getLocationString());
    }
}
