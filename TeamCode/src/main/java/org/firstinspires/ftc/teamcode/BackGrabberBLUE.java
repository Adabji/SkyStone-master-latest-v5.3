package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.finalAutoHeading;

@Config
@Autonomous(name = "BackGrabberBLUE", group = "Autonomous")
public class BackGrabberBLUE extends LinearOpMode {
    // CV stuff
    private OpenCvCamera phoneCam;
    private TESTSkystoneDetector skyStoneDetector;
    String skystoneLoc = "";
    public static int skystoneMargin = 120;
    public static int cameraRightMargin = 210;

    double landingHeading = 0;

    // hardware stuff

    // movement stuff
    public static double movementToCenter = 0;
    public static double movementToLeft = 12;
    public static double movementToRight = 4;

    public static double initX = 0;
    public static double initY = 0;
    public static double movementb2;
    public static double stoneX = movementb2;
    public static double stoneY = -41.5;
    public static double initHeading = 0;
    public static double turnAngle = Math.toRadians(88);
    public static double movementa1;
    public static double movementc3;
    public static double movementd4;
    public static double movemente5;
    public static double movementf6;
    public static double movementg7;
    public static double movementh8 = -89;
    public static double movementi9;
    public static double movementl12;
    public static double movementn14;
    public static double movementk11;
    public static double movementm13 = 10;
    public static double movementu21 = 23;
    public static double movementt20;
    public static double movemento15 = 83;
    public static double movementp16 = 3.03;    // turn
    public static double movementq17 = 8;
    public static double movementv22;
    public static double movementw23;

    // Timers
    double detectionTimer = -1;
    double bufferTimer = -1;
    public static long timer1 = 0;
    public static long timer2 = 2100;
    // public static double movements19 = 30;

    // Hardware stuff
    private Servo foundationServo, foundationServoRight, rightStoneGrabber, grabberLeft, tapeMeasure, liftHoExt, wrist, grabber;
    private DcMotor intakeMotor1, intakeMotor2, intakeMotor3;
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
        tapeMeasure = hardwareMap.servo.get("tapeMeasure");
        liftHoExt = hardwareMap.servo.get("liftHoExt");
        wrist = hardwareMap.servo.get("liftGrabberRotater");
        grabber = hardwareMap.servo.get("liftGrabber");
        intakeMotor1 = hardwareMap.dcMotor.get("intake motor 1");
        intakeMotor2 = hardwareMap.dcMotor.get("intake motor 2");
        intakeMotor3 = hardwareMap.dcMotor.get("intake motor 3");
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor3.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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

            if (skyStoneDetector.getScreenPosition().x > cameraRightMargin || skyStoneDetector.getScreenPosition().x < 10) {
                skystoneLoc = "right";
                movementb2 = 3;
                movementl12 = 100;
                movementg7 = 90;
                movementv22 = 120;
                movemente5 = 9;
                movementf6 = 85.5;
                movementn14 = 103;
                movementi9 = 14.5;
                movementk11 = 0;
                movementa1 = 33;
                movementc3 = 14;
                movementw23 = 0;
                movementd4 = 36;
                movementt20 = 120;
            } else if (skyStoneDetector.getScreenPosition().x < skystoneMargin) {
                skystoneLoc = "left";
                movementb2 = -8;
                movementl12 = 72;
                movementg7 = 100;
                movementv22 = 96;
                movemente5= 11.5;
                movementf6 = 85;
                movementn14 = 51;
                movementi9 = 14.5;
                movementk11 = 45;
                movementa1 = 0;
                movementc3 = 30;
                movementw23 = 23;
                movementd4 = 0;
                movementt20 = 63;
            } else {
                skystoneLoc = "center";
                movementg7 = 83;
                movementb2 = 12;
                movementl12 = 90;
                movementv22 = 115;
                movemente5 = 7;
                movementf6 = 85;
                movementn14 = 93.5;
                movementi9 = 14.5;
                movementk11 = 0;
                movementa1 = 33  ;
                movementc3 = 11.5;
                movementw23 = 0;
                movementd4 = 36;
                movementt20 = 110;
            }

            telemetry.addData("Skystone Location", skystoneLoc);
            telemetry.addData("Skystone coordinates", skyStoneDetector.getScreenPosition());
            telemetry.update();

            foundationDownGrabberUp();
            readyToGrab();
            extensionIn();
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
            grabFoundation();
            sleep(350);
            moveForward(drive,15);
            sleep(300);
            while(drive.getExternalHeading() < movementp16) { drive.setMotorPowers(-0.16, -0.16, 0.7, 0.7); }
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setPoseEstimate(new Pose2d (0, 0, 0));
            releaseFoundation();
            moveBackward(drive,20);
            strafeLeft(drive,14);
            intakeMotor1.setPower(1);
            sleep(400);
            intakeMotor1.setPower(0);
            moveForward(drive,movementn14);
            rotate(drive,movementk11);
            strafeLeft(drive,movementa1);
            intakeOn();
            moveForward(drive,movementc3);
            moveBackward(drive,movementw23);
            rotate(drive,-movementk11);
            strafeRight(drive,movementd4);
            intakeReverse();
            grabStoneInRobot();
            sleep(300);
            intakeOff();
            tapeMeasure.setPosition(.325);
            moveBackward(drive,movementt20);
            extensionOut();
            sleep(1000);
            releaseStone();



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

            /*if (drive.getExternalHeading() > 0 && drive.getExternalHeading() < Math.PI) {
                finalAutoHeading = -drive.getExternalHeading();
            } else {
                finalAutoHeading = (2*Math.PI) - drive.getExternalHeading();
            }*/
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
        foundationServoRight.setPosition(.63);  // originally .6
        grabberLeft.setPosition(.7);
    }
    private void foundationDownGrabberDown(){
        foundationServoRight.setPosition(.95);
        grabberLeft.setPosition(.78);
    }
    private void foundationAndStoneAllIn(){
        foundationServoRight.setPosition(.3);
        grabberLeft.setPosition(.7);
    }
    private void grabFoundation() {
        foundationServoRight.setPosition(1);
        foundationServo.setPosition(0.25);
        rightStoneGrabber.setPosition(.8);
        grabberLeft.setPosition(.28);
    }
    private void releaseFoundation() {
        foundationServoRight.setPosition(.5);
        foundationServo.setPosition(0.75);
    }
    private void foundationDownGrabberUp2(){
        if(skystoneLoc!= "left"){
            foundationServoRight.setPosition(0.95);
            grabberLeft.setPosition(.3);
            }
        else {
            foundationServo.setPosition(0.3);
            rightStoneGrabber.setPosition(.8);
        }
    }
    private void foundationUpGrabberDown2(){
        if(skystoneLoc!= "left"){
            foundationServoRight.setPosition(.63);  // originally .6
            grabberLeft.setPosition(.7);
            }
        else{
            foundationServo.setPosition(.62);   // originally .75
            rightStoneGrabber.setPosition(.35);
        }
    }
    private void foundationDownGrabberDown2(){
        if(skystoneLoc!= "left"){
            foundationServoRight.setPosition(.95);
            grabberLeft.setPosition(.7);
            }
        else{
            foundationServo.setPosition(0.3);
            rightStoneGrabber.setPosition(.35);
        }
    }
    private void intakeOn() {
        intakeMotor1.setPower(1);
        intakeMotor2.setPower(1);
        intakeMotor3.setPower(1);
    }
    private void intakeOff() {
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
        intakeMotor3.setPower(0);
    }
    private void intakeReverse() {
        intakeMotor1.setPower(-1);
        intakeMotor2.setPower(-1);
        intakeMotor3.setPower(1);
    }
    private void readyToGrab(){
        grabber.setPosition(0.44);
        wrist.setPosition(0.1);
    }
    private void grabStoneInRobot(){
        grabber.setPosition(0.4);
        wrist.setPosition(0.5);
    }
    private void extensionIn(){
        liftHoExt.setPosition(0.575);
    }
    private void extensionOut(){
        liftHoExt.setPosition(1);
    }
    private void releaseStone(){
        grabber.setPosition(0.8);
        wrist.setPosition(0.1);
    }
}
