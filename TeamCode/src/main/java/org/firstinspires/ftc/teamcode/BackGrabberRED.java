package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(name = "BackGrabberRED", group = "Autonomous")
public class BackGrabberRED extends LinearOpMode {
    // CV stuff
    private OpenCvCamera phoneCam;
    private TESTSkystoneDetector skyStoneDetector;
    String skystoneLoc = "";
    public static int skystoneMargin = 145;
    public static int cameraRightMargin = 210;

    public static int redLeftMargin = 70;
    public static int redCenterMargin = 160;
    public static int redRightMargin = 8;

    double landingHeading = 0;

    // hardware stuff
    private static CRServo tapeMeasure;

    // movement stuff
    public static double movementToCenter = 0;
    public static double movementToLeft = 12;
    public static double movementToRight = 4;

    public static double initX = 0;
    public static double initY = 0;
    public static double movementb2;
    public static double stoneX = movementb2;
    public static double stoneY = -40;
    public static double initHeading = 0;
    public static double turnAngle = Math.toRadians(88);
    public static double movementa1 = 20;
    public static double movementc3 = 87.5;
    public static double movementd4 = 25;
    public static double movemente5 = 10;
    public static double movementf6;
    public static double movementg7 = 83;
    public static double movementh8 = 89;
    public static double movementi9 = 12;
    public static double movementl12 = 84;
    public static double movementn14 = 16.5;
    public static double movementk11 = 12;
    public static double movementm13 = 9;
    public static double movementt20 = 7;
    public static double movemento15 = -92;
    public static double movementp16 = 0.11115;    // turn
    public static double movementq17 = -10;
    public static double movementv22;

    // Timers
    double detectionTimer = -1;
    double bufferTimer = -1;
    public static long timer1 = 0;
    public static long timer2 = 2100;
    // public static double movements19 = 30;

    // Hardware stuff
    private Servo foundationServo, foundationServoRight, rightStoneGrabber, grabberLeft;
    public DriveConstraints constraints = new DriveConstraints(
            60.0, 40.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        foundationServo = hardwareMap.servo.get("foundationServoLeft");
        foundationServoRight = hardwareMap.servo.get("foundationServoRight");
        rightStoneGrabber = hardwareMap.servo.get("rightStoneGrabber");
        grabberLeft = hardwareMap.servo.get("grabberLeft");
        tapeMeasure = hardwareMap.crservo.get("tapeMeasure");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        skyStoneDetector = new TESTSkystoneDetector();
        skyStoneDetector = new TESTSkystoneDetector();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Status", "Waiting for start command...");
            // telemetry.addData("Heading", drive.getExternalHeading());
            // telemetry.addData("Skystone Location", skystoneLoc);
            // telemetry.addData("Skystone coordinates", skyStoneDetector.getScreenPosition());
            telemetry.update();
        }
        if (opModeIsActive() && !isStopRequested()) {

            skyStoneDetector.setFoundToFalse();
            detectionTimer = System.currentTimeMillis();

            while (System.currentTimeMillis() - detectionTimer < 1000) { }

            if (skyStoneDetector.getScreenPosition().x > redRightMargin && skyStoneDetector.getScreenPosition().x < redLeftMargin) {
                skystoneLoc = "right";
                movementb2 = -23;
                movementg7 = 70;
                movementl12 = 74;
                movementf6 = -88;
                movementv22 = 108;
            } else if (skyStoneDetector.getScreenPosition().x > redLeftMargin && skyStoneDetector.getScreenPosition().x < redCenterMargin) {
                skystoneLoc = "left";
                movementb2 = 3.5;
                movementg7 = 92.5;
                movementl12 = 96;
                movementf6 = -89.5;
                movementv22 = 125;
            } else if (skyStoneDetector.getScreenPosition().x > redCenterMargin || skyStoneDetector.getScreenPosition().x < 2) {
                skystoneLoc = "center";
                movementb2 = -8;
                movementg7 = 83;
                movementl12 = 83;
                movementf6 = -88;
                movementv22 = 115;
            }

            telemetry.addData("Skystone Location", skystoneLoc);
            telemetry.addData("Skystone coordinates", skyStoneDetector.getScreenPosition());
            telemetry.update();

            foundationDownGrabberUp();
            TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(new Pose2d(initX, initY, initHeading), constraints);

            // trajectoryBuilder.splineTo(new Pose2d(finalX, finalY, finalHeading));

            trajectoryBuilder.lineTo(new Vector2d(movementb2, stoneY), new LinearInterpolator(initHeading, turnAngle));

            drive.followTrajectorySync(trajectoryBuilder.build());
            foundationDownGrabberDown();
            moveForward(drive,movemente5);
            foundationUpGrabberDown();
            rotate(drive,movementf6);
            moveBackward(drive,movementg7);
            rotate(drive,movementh8);
            moveBackward(drive,movementi9);
            foundationDownGrabberUp();
            sleep(300);
            foundationUpGrabberDown();
            strafeLeft(drive,movementq17);
            // moveForward(drive,5);
            grabFoundation();
            moveBackward(drive,4);
            sleep(300);
            while(drive.getExternalHeading() > movementp16) { drive.setMotorPowers(0.7, 0.7, -0.05, -0.05); }
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setPoseEstimate(new Pose2d (0, 0, 0));
            releaseFoundation();
            moveForward(drive,movementl12);
            foundationDownGrabberUp();
            strafeLeft(drive,movementt20);
            rotate(drive,movementh8);
            moveBackward(drive,movementn14);
            foundationDownGrabberDown();
            moveForward(drive,movementm13);
            foundationUpGrabberDown();
            rotate(drive,movemento15);
            sleep(100);
            moveBackward(drive,movementv22);

            // tapeMeasure.setPower(power);
            // +power = tape measure retracts
            // -power = tape measure extends

            /*moveForward(drive,movementi9);
            rotate(drive,movementj10);
            moveBackward(drive,movementg7+movementb2+movementk11);
            rotate(drive,movementl12);
            foundationDownGrabberUp();
            moveBackward(drive,movementm13);
            foundationDownGrabberDown();
            moveForward(drive,move
            moveBackward(drive,movementg7+movementb2+movementk11+movementt20);
            rotate(drive,movemento15);mentn14);
            foundationUpGrabberDown();
            rotate(drive,movementf6);
            moveBackward(drive,movementp16);
            foundationDownGrabberUp();
            //sleep(200);
            strafeLeft(drive,movementq17);
            grabFoundation();
            while(drive.getExternalHeading() > 1.571) { drive.setMotorPowers(0.5, 0.5, 0, 0); }
            drive.setMotorPowers(0, 0, 0, 0);
            releaseFoundation();
            strafeRight(drive,movementr18);
            moveForward(drive,movements19);*/

        }
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
    private void foundationDownGrabberUp(){
        foundationServo.setPosition(0.32);
        rightStoneGrabber.setPosition(.7);
    }
    private void foundationUpGrabberDown(){
        foundationServo.setPosition(.72);   // originally .75
        rightStoneGrabber.setPosition(.4);
    }
    private void foundationDownGrabberDown(){
        foundationServo.setPosition(0.32);
        rightStoneGrabber.setPosition(.4);
    }
    private void foundationAndStoneAllIn(){
        foundationServo.setPosition(.3);
        rightStoneGrabber.setPosition(.7);
    }
    private void grabFoundation() {
        foundationServoRight.setPosition(1);
        foundationServo.setPosition(0.25);
        rightStoneGrabber.setPosition(.7);
        grabberLeft.setPosition(.3);

    }
    private void releaseFoundation() {
        foundationServoRight.setPosition(.5);
        foundationServo.setPosition(0.75);
    }
}

