package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.revextensions2.ExpansionHubMotor;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import java.util.Locale;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */

@Disabled
@Autonomous(name = "Skystone Detector OpMode", group="Autonomous")
public class SkystoneDetectorExample extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    // hardware stuff
    private static DcMotor intakeMotor, liftMotor1;
    private static ExpansionHubMotor intakeMotorRE2, liftRE2;
    private static DcMotorEx liftEx1;
    private static Servo leftElbow, rightElbow, wrist, grip, foundationServoLeft, foundationServoRight;
    private TouchSensor liftTouch;
    private TouchSensor intakeTouch;
    double                  globalAngle, power = .30, correction, rotation;
    double                  rotationPower = 0.5;
    double                  movePower = 0.7;
    Orientation             lastAngles = new Orientation();
    private DistanceSensor colorSensorDistance;

    // PIDF stuff
    PIDFCoefficients pidfCoefficients;
    double lkp = 6;
    double lki = 0;
    double lkd = 0;
    double lkf = 0;

    // encoder things
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.937;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);   // 43.4667


    static final double LIFT_COUNTS_PER_MOTOR_REV = 537.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = 1.0;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);  // 136.90275

    // Camera stuff
    String skystoneLoc = "";

    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        // hardware stuff


        /*
         * Wait for the user to press start on the Driver Station
         */
        while (!opModeIsActive() && !isStopRequested()) {
            if (skyStoneDetector.isDetected()) {
                if (skyStoneDetector.getScreenPosition().x < 100) {
                    skystoneLoc = "left";
                } else if (skyStoneDetector.getScreenPosition().x < 190 && skyStoneDetector.getScreenPosition().x > 100) {
                    skystoneLoc = "center";
                } else {
                    skystoneLoc = "right";
                }
            }

            telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition().x, skystoneLoc);
            telemetry.addData("Status", "Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition().x, skystoneLoc);
            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            // telemetry.addData("Frame Count", phoneCam.getFrameCount());
            // telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            // telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            // telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            // telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            // telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            // FIRST SKYSTONE - OUTER ONE
            /*move(1, movePower/2, true);
            if (skystoneLoc.equals("right")) {
                strafe(11, movePower/1.5, true);
            } else if (skystoneLoc.equals("center")) {
                strafe(3, movePower/1.5, true);
            }

            rotate(90, rotationPower);
            strafe(25, movePower, true);
            move(1, movePower/2, true);*/

            do {
                intakeMotor.setPower(-1);
            } while (!intakeTouch.isPressed());

            intakeMotor.setPower(0);
            // elbow out

            // move until foundation in sight
            // WHAT IF FOUNDATION IS NOT MOVED
            // FOCUS ON JUST MOVING THEM FIRST
            /*
             * if (inchesc > 44 or something) {
             *  turnLeft 90 deg
             *  foundation Servo Up
             *  moveForward a bit
             *  foundation Servo Down
             *  pull back in a curve
             *  push forward
             *  release grabber
             *  go for second stone or stop on the line
             * }
             */

            // deposit stone
        }
    }
}