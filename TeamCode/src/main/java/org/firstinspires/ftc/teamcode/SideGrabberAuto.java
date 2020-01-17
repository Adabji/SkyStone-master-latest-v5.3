package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.TextFileAuto;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.finalAutoHeading;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */

@Autonomous(name = "Auto RED", group="Autonomous")
public class SideGrabberAuto extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private LastHopeDetector skyStoneDetector;

    private DcMotorEx intakeMotor1, intakeMotor2;
    private static DcMotorEx liftEx1;
    private Servo foundationServoLeft, foundationServoRight, sideGrabberServo1, sideGrabberServo2;

    int currentLiftStage = 0;
    int targetPos = 0;
    double liftPower = 1;
    long liftUpTimer = -1;
    long sideGrabTimer = -1;

    // Camera stuff
    String skystoneLoc = "";

    // hardware stuff
    private static Servo liftHoExt, wrist, grabber, stoneHolder;
    private TouchSensor liftTouch;
    private DistanceSensor intakeColor;

    // PIDF stuff
    PIDFCoefficients pidfCoefficients;
    double lkp = 6;
    double lki = 0;
    double lkd = 0;
    double lkf = 0;

    // dashboard stuff
    public static int leftAndCenterBlockMargin = 80;
    public static double strafeDist = 2;
    public static double moveToStoneDist = 40;
    public static double centerStrafe = 5;
    public static double moveABitToStone = 0;
    public static double moveToIntakeStone = 9;
    public static double strafeSideGrabStone = 7;
    public static double screenYPos = 160;
    public static double strafeToFoundationDist = 33;
    public static double moveBackToFoundationDist = 15;
    public static double moveToLine = 35;

    DriveConstraints stoneCollectionConstraints = new DriveConstraints(10.0, 10.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0);
    @Override
    public void runOpMode() {
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake motor 1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake motor 2");
        liftEx1 = hardwareMap.get(DcMotorEx.class, "lift motor 1");

        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");
        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");
        sideGrabberServo1 = hardwareMap.get(Servo.class, "sideGrabberServo1");
        sideGrabberServo2 = hardwareMap.get(Servo.class, "sideGrabberServo2");
        liftHoExt = hardwareMap.servo.get("liftHoExt");
        wrist = hardwareMap.servo.get("liftGrabberRotater");
        grabber = hardwareMap.servo.get("liftGrabber");
        stoneHolder = hardwareMap.servo.get("stoneHolder");

        intakeColor = hardwareMap.get(DistanceSensor.class, "intakeColor");
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");

        pidfCoefficients = new PIDFCoefficients(lkp, lki, lkd, lkf);

        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Servo positions
        liftHoExt.setPosition(0.45);
        wrist.setPosition(0.18);
        grabber.setPosition(0.6);
        foundationServosUp();

        // Side Grabber Servo positions Up
        sideGrabberServosUp();

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new LastHopeDetector();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        while (!opModeIsActive() && !isStopRequested()) {
            if (skyStoneDetector.isDetected()/* && skyStoneDetector.foundRectangle().y > 180*/) {
                if (skyStoneDetector.foundRectangle().x < 80) {
                    skystoneLoc = "left";
                    strafeDist = 15;
                    moveBackToFoundationDist = 43;
                } else if (skyStoneDetector.foundRectangle().x < 150 && skyStoneDetector.foundRectangle().x > 80) {
                    skystoneLoc = "center";
                    strafeDist = 2;
                    moveBackToFoundationDist = 33;
                } else {
                    skystoneLoc = "right";
                    strafeDist = 6;
                    moveBackToFoundationDist = 30;
                }
            }

            telemetry.addData("Skystone Location = " + skyStoneDetector.foundRectangle().x, skystoneLoc);
            telemetry.addData("Status", "Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            phoneCam.stopStreaming();

            // open intake motors
            intakeMotor1.setPower(1);
            intakeMotor2.setPower(-1);
            sleep(50);
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);

            // telemetry.addData("Current heading", Math.toDegrees(drive.getExternalHeading()));
            // telemetry.update();

            moveForward(drive, moveToStoneDist);

            // FIRST SKYSTONE - OUTER ONE
            if (skystoneLoc.equals("right")) {
                strafeRight(drive, strafeDist);
            } else if (skystoneLoc.equals("center")) {
                strafeLeft(drive, strafeDist);
            } else if (skystoneLoc.equals("left")) {
                strafeLeft(drive, strafeDist);
            } else {
                strafeLeft(drive, strafeDist);
            }

            // moveForward(drive, moveToStoneDist);
            rotate(drive, 90);
            strafeRight(drive, strafeSideGrabStone);
            sleep(300);

            // grab skystone using the side grabber
            grabSkyStone();

            sleep(200);

            sideGrabberServo1.setPosition(0.61);

            strafeLeft(drive, strafeSideGrabStone);
            moveBackward(drive, 50 + moveBackToFoundationDist);
            strafeRight(drive, 7);

            // side grabber elbow down 1
            // side grabber elbow up 0.61
            // side grabber down 0.81
            // side grabber up 0.47

            // deposit skystone
            sideGrabberServo1.setPosition(1);
            sideGrabberServo2.setPosition(0.47);

            strafeLeft(drive, 10);
        }
    }
    
    private void sideGrabberServosUp() {
        sideGrabberServo1.setPosition(0.61);
        sideGrabberServo2.setPosition(0.47);
    }

    private void grabSkyStone() {
        sideGrabberServo1.setPosition(1);
        sleep(300);
        sideGrabberServo2.setPosition(0.81);
    }

    private void extendStone() {
        currentLiftStage = 2;
        liftUpTimer = System.currentTimeMillis();

        while (System.currentTimeMillis() - liftUpTimer > 800) { }

        liftHoExt.setPosition(0.92);
        sleep(200);
        wrist.setPosition(0.18);
        currentLiftStage = 0;
    }

    private void retractExt() {
        wrist.setPosition(0.18);
        grabber.setPosition(0.32);
        sleep(200);
        liftHoExt.setPosition(0.45);
    }

    private void startIntakeMotors(double p) {
        intakeMotor1.setPower(p);
        intakeMotor2.setPower(p);
    }

    private void stopIntakeMotors() {
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
    }

    private void moveForward(SampleMecanumDriveBase thisDrive, double distance) {
        thisDrive.followTrajectorySync(thisDrive.trajectoryBuilder().forward(distance).build());
    }

    private void moveBackward(SampleMecanumDriveBase thisDrive, double distance) {
        thisDrive.followTrajectorySync(thisDrive.trajectoryBuilder().back(distance).build());
    }

    private void strafeLeft(SampleMecanumDriveBase thisDrive, double distance) {
        thisDrive.followTrajectorySync(thisDrive.trajectoryBuilder().strafeLeft(distance).build());
    }

    private void strafeRight(SampleMecanumDriveBase thisDrive, double distance) {
        thisDrive.followTrajectorySync(thisDrive.trajectoryBuilder().strafeRight(distance).build());
    }

    private void splineBotTo(SampleMecanumDriveBase thisDrive, double x, double y, double heading) {
        thisDrive.followTrajectorySync(thisDrive.trajectoryBuilder().splineTo(new Pose2d(x, y, heading)).build());
    }

    private void rotate(SampleMecanumDriveBase thisDrive, double angleInDeg) {
        thisDrive.turnSync(Math.toRadians(angleInDeg));
    }

    private void foundationServosDown() {
        foundationServoLeft.setPosition(0.21);
        foundationServoRight.setPosition(0.95);
    }

    private void foundationServosUp() {
        foundationServoLeft.setPosition(0.77);
        foundationServoRight.setPosition(0.37);
    }
}