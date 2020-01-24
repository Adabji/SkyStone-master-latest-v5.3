package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(name = "Back Grabber With CV BLUE", group = "Autonomous")
public class BackGrabberWithCVBLUE extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private TESTSkystoneDetector skyStoneDetector;

    double landingHeading = 0;

    public static double movementa1 = 20;
    public static double movementb2 = 4;

    public static double movementToCenter = 4;
    public static double movementToLeft = 0;
    public static double movementToRight = 12;

    public static double movementc3 = 90;
    public static double movementd4 = 25;
    public static double movemente5 = 10;
    public static double movementf6 = 88;
    public static double movementg7 = 85;
    public static double movementh8 = -90;
    public static double movementi9 = 12;
    public static double movementl12 = 87;
    public static double movementn14 = 6;
    public static double movementk11 = 20;
    public static double movementm13 = 98;
    public static double movementu21 = 12;
    public static double movementt20 = 15;
    public static double movemento15 = 85;
    /*  public static double movementj10 = -92;
      public static double movementp16 = 15;*/
    public static double movementq17 = 4;
    public static double movementr18 = 10;
    // public static double movements19 = 30;

    String skystoneLoc = "";
    public static int skystoneMargin = 145;
    public static int cameraRightMargin = 210;

    // Timers
    double detectionTimer = -1;


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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        skyStoneDetector = new TESTSkystoneDetector();

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        skyStoneDetector.setFoundToFalse();

        while (!opModeIsActive() && !isStopRequested()) {
            // skyStoneDetector.setFoundToFalse();

            // detectionTimer = System.currentTimeMillis();

            // while (System.currentTimeMillis() - detectionTimer < 500) { }

            // telemetry.addData("Skystone ScreenPos", skyStoneDetector.getScreenPosition());
            // telemetry.update();
            // sleep(500);

            if (skyStoneDetector.getScreenPosition().x > cameraRightMargin) {
                skystoneLoc = "right";
                movementb2 = movementToRight;
            } else if (skyStoneDetector.getScreenPosition().x < skystoneMargin) {
                skystoneLoc = "left";
                movementb2 = movementToLeft;
            } else {
                skystoneLoc = "center";
                movementb2 = movementToCenter;
            }

            telemetry.addData("Skystone Location", skystoneLoc);
            telemetry.addData("Skystone coordinates", skyStoneDetector.getScreenPosition());
            telemetry.addData("Status", "Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            /*phoneCam.openCameraDevice();
            phoneCam.setPipeline(skyStoneDetector);
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            skyStoneDetector.setFoundToFalse();

            detectionTimer = System.currentTimeMillis();

            while (System.currentTimeMillis() - detectionTimer < 500) { }

            telemetry.addData("Skystone ScreenPos", skyStoneDetector.getScreenPosition());
            telemetry.update();
            sleep(500);

            if (skyStoneDetector.getScreenPosition().x < 50) {
                skystoneLoc = "left";
                movementb2 = movementToLeft;
            } else if (skyStoneDetector.getScreenPosition().x < 150 && skyStoneDetector.getScreenPosition().x > 50) {
                skystoneLoc = "center";
                movementb2 = movementToCenter;
            } else {
                skystoneLoc = "right";
                movementb2 = movementToRight;
            }*/

            /**
             * ROTATION WITH GYRO TEST
             */

            while (drive.getExternalHeading() != getLandingHeading(drive, Math.toRadians(90))) { drive.setMotorPowers(-0.7, -0.7, 0.7, 0.7); }
            while (drive.getExternalHeading() != getLandingHeading(drive, Math.toRadians(-180))) { drive.setMotorPowers(0.7, 0.7, -0.7, -0.7); }


            /*strafeRight(drive, movementa1);
            moveBackward(drive, movementb2);
            rotate(drive, movementc3);
            foundationDownGrabberUp();
            moveBackward(drive,movementd4);
            foundationDownGrabberDown();
            moveForward(drive,movemente5);
            foundationUpGrabberDown();
            rotate(drive,movementf6);
            moveBackward(drive,movementg7+movementb2);
            rotate(drive,movementh8);
            moveBackward(drive,movementi9);
            foundationDownGrabberUp();
            sleep(300);
            foundationUpGrabberDown();
            strafeLeft(drive,movementq17);
            // moveForward(drive,5);
            grabFoundation();
            moveBackward(drive,4);
            sleep(500);
            while(drive.getExternalHeading() < 3.1415) { drive.setMotorPowers(0, 0, 0.7, 0.7); }
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setPoseEstimate(new Pose2d (0, 0, 0));
            releaseFoundation();
            moveForward(drive,movementl12);
            foundationDownGrabberUp();
            strafeRight(drive,movementt20);
            rotate(drive,movementh8);
            strafeLeft(drive,movementu21);
            moveBackward(drive,movementn14);
            foundationDownGrabberDown();
            moveForward(drive,movementn14);
            foundationUpGrabberDown();
            rotate(drive,movemento15);
            //moveBackward(drive,movementm13);
            drive.setMotorPowers(-1, -1, -1, -1);
            sleep(1700);
            drive.setMotorPowers(0, 0, 0, 0);
            foundationDownGrabberUp();
            sleep(300);
            foundationUpGrabberDown();
            moveForward(drive,movementk11);*/

            /*moveForward(drive,movementi9);
            rotate(drive,movementj10);
            moveBackward(drive,movementg7+movementb2+movementk11);
            rotate(drive,movementl12);
            foundationDownGrabberUp();
            moveBackward(drive,movementm13);
            foundationDownGrabberDown();
            moveForward(drive,movementn14);
            foundationUpGrabberDown();
            rotate(drive,movementf6);
            moveBackward(drive,movementg7+movementb2+movementk11+movementt20);
            rotate(drive,movemento15);
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
        foundationServoRight.setPosition(0.95);
        grabberLeft.setPosition(.3);
    }
    private void foundationUpGrabberDown(){
        foundationServoRight.setPosition(.6);
        grabberLeft.setPosition(.7);
    }
    private void foundationDownGrabberDown(){
        foundationServoRight.setPosition(1);
        grabberLeft.setPosition(.7);
    }
    private void foundationAndStoneAllIn(){
        foundationServoRight.setPosition(.3);
        grabberLeft.setPosition(.7);
    }
    private void grabFoundation() {
        foundationServoRight.setPosition(1);
        foundationServo.setPosition(0.25);
    }
    private void releaseFoundation() {
        foundationServoRight.setPosition(.5);
        foundationServo.setPosition(0.75);
    }

    // turn right is negative turnAngle, turn left is positive turnAngle
    private double getLandingHeading(SampleMecanumDriveBase thisDrive, double turnAngle) {
        double currentHeading = thisDrive.getExternalHeading();
        landingHeading = currentHeading + turnAngle;
        if (currentHeading > 0 && landingHeading < 0) {
            landingHeading += 2*Math.PI;
        } else if (currentHeading > 0 && landingHeading > 2*Math.PI) {
            landingHeading -= 2*Math.PI;
        }
        return landingHeading;
    }
}
