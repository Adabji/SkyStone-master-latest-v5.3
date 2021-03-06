package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;



import kotlin.Unit;

@Config
@Autonomous(name = "BackGrabberREDSpline", group = "Autonomous")
public class BackGrabberREDSpline extends LinearOpMode {
    // CV stuff
    private OpenCvCamera phoneCam;
    private TESTSkystoneDetector skyStoneDetector;
    String skystoneLoc = "";
    public static int skystoneMargin = 120;
    public static int cameraRightMargin = 210;
    double liftPower = 1;
    PIDFCoefficients pidfCoefficients;

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
    public static double movementm13;
    public static double movementu21 = 23;
    public static double movementt20;
    public static double movemento15 = 83;
    public static double movementp16 = .11115;    // turn
    public static double movementq17 = 8;
    public static double movementv22;
    public static double movementw23;
    static final double LIFT_COUNTS_PER_MOTOR_REV = 537.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = .5;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);
    double lkp = 6;
    double lki = 0;
    double lkd = 0;
    double lkf = 0;


    // Timers
    double detectionTimer = -1;
    double bufferTimer = -1;
    public static long timer1 = 0;
    public static long timer2 = 2100;
    // public static double movements19 = 30;

    // Hardware stuff
    private Servo foundationServo, foundationServoRight, rightStoneGrabber, grabberLeft, tapeMeasure, liftHoExt, wrist, grabber, capstoneServo;
    private DcMotor intakeMotor1, intakeMotor2, intakeMotor3;
    private static DcMotorEx liftEx1;

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
        capstoneServo = hardwareMap.servo.get("capstoneServo");
        intakeMotor1 = hardwareMap.dcMotor.get("intake motor 1");
        intakeMotor2 = hardwareMap.dcMotor.get("intake motor 2");
        intakeMotor3 = hardwareMap.dcMotor.get("intake motor 3");
        liftEx1 = hardwareMap.get(DcMotorEx.class, "lift motor 1");
        pidfCoefficients = new PIDFCoefficients(lkp, lki, lkd, lkf);
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor3.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setDirection(DcMotorSimple.Direction.REVERSE);

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
                movementb2 = 4;
                movementl12 = -29;
                movementg7 = -9.2;
                movementv22 = 95.3;
                movemente5= 11.5;
                movementf6 = -2.75;
                movementn14 = 51;
                movementi9 = 90;
                movementk11 = 58;
                movementa1 = 60;
                movementc3 = 32.3;
                movementw23 = 23;
                movementd4 = -109.5;
                movementt20 = 60;
                movementm13 = -4.8;

            } else if (skyStoneDetector.getScreenPosition().x < skystoneMargin) {
                skystoneLoc = "left";
                movementb2 = 7.5;
                movementl12 = -26.7;
                movementg7 = -11;
                movementv22 = 118.6;
                movemente5 = 9;
                movementf6 = -3;
                movementn14 = 103;
                movementi9 = 90;
                movementk11 = 65;
                movementa1 = 60;
                movementc3 = 39;
                movementw23 = 26;
                movementd4 = -98;
                movementt20 = 120;
                movementm13 = -5;
            } else {
                skystoneLoc = "center";
                movementg7 = -9.2;
                movementb2 = -4;
                movementl12 = -29;
                movementv22 = 102.2;
                movemente5 = 7;
                movementf6 = -4.5;
                movementn14 = 93.5;
                movementi9 = 90;
                movementk11 = 58;
                movementa1 = 60;
                movementc3 = 34.5;
                movementw23 = 0;
                movementd4 = -86.5;
                movementm13 = -5.5;
            }

            telemetry.addData("Skystone Location", skystoneLoc);
            telemetry.addData("Skystone coordinates", skyStoneDetector.getScreenPosition());
            telemetry.update();

            capstoneServo.setPosition(0.5);
            foundationDownGrabberUp2();
            readyToGrab();
            extensionIn();
            drive.setPoseEstimate(new Pose2d (0, 0, 0));
            TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(new Pose2d(initX, initY, initHeading), constraints);

            // trajectoryBuilder.splineTo(new Pose2d(finalX, finalY, finalHeading));

            trajectoryBuilder.lineTo(new Vector2d(movementb2, stoneY), new LinearInterpolator(initHeading, turnAngle));

            drive.followTrajectorySync(trajectoryBuilder.build());
           // strafeRight(drive,17);
            //drive.followTrajectorySync(
            // drive.trajectoryBuilder()
           // .splineTo(new Pose2d(-8,-37.5,Math.toRadians(88)))
                   /*.addMarker(() -> {
                        return Unit.INSTANCE;
                    })*/
                 // .build());
            foundationDownGrabberDown2();
            sleep(450);
            foundationUpGrabberDown2();
            // grabFoundation();  *you can add commands in between too*
            sleep(500);
            drive.followTrajectorySync(
            drive.trajectoryBuilder()
                    .setReversed(true)
                    .splineTo(new Pose2d(-26,-31.7,Math.toRadians(0)))
                    .splineTo(new Pose2d(-76,-31.7,Math.toRadians(0)))
                    .splineTo(new Pose2d(-95,-46.5,Math.toRadians(91.5)))
                    .build());
            grabFoundation();
            sleep(500);
            moveForward(drive,15);
            while(drive.getExternalHeading() > movementp16) { drive.setMotorPowers(0.7, 0.7, -0.16, -0.16); }
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setPoseEstimate(new Pose2d (0, 0, 0));
            releaseFoundation();
            moveBackward(drive,26);
            drive.followTrajectorySync(
                    // drive towards 2nd stone
            drive.trajectoryBuilder()
                    .splineTo(new Pose2d(27,-8, 0))
                    .addMarker(() -> {
                        intakeMotor1.setPower(1);
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(movementa1,movementm13,Math.toRadians(0)))
                    //passing middle bridge
                    .addMarker(() -> {
                        intakeOn();
                        //grabStoneInRobot();
                        return Unit.INSTANCE;
                    })
                    // going for 2nd stone
                    .splineTo(new Pose2d(movementv22,movementl12,Math.toRadians(-45)))
                    .build());
            sleep(10);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            //getting 2nd stone
                    .splineTo(new Pose2d(movementi9,movementf6,0))
                    .addMarker(() -> {
                        intakeReverse();
                        grabStoneInRobot();
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(-10,movementf6))
            .build());
            intakeOff();
            liftHeight(2);
            extensionOut();
            sleep(600);
            releaseStone();
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(7,movementg7))
                            .addMarker(() -> {
                                extensionIn();
                                readyToGrab();
                                liftHeight(0);
                                return Unit.INSTANCE;
                            })
                            .splineTo(new Pose2d(movementk11,movementg7,Math.toRadians(-25)))
                            .addMarker(() -> {
                                intakeOn();
                                return Unit.INSTANCE;
                            })
                            .splineTo(new Pose2d(-movementd4,-movementc3,Math.toRadians(-45)))
                            .build());
            sleep(10);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(movementk11,-4.7,0))
                            .addMarker(() -> {
                                intakeReverse();
                                grabStoneInRobot();
                                tapeMeasure.setPosition(0.25);
                                return Unit.INSTANCE;
                            })
                            .splineTo(new Pose2d(20,-4.7,0))
                            .addMarker(() -> {
                                intakeOff();
                                liftHeight(2);
                                extensionOut();
                                return Unit.INSTANCE;
                            })
                            .splineTo(new Pose2d(-13,-4.7,0))
                            .build());
            releaseStone();
            extensionIn();
            tapeMeasure.setPosition(0.25);
            moveForward(drive, 20);
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
        foundationServo.setPosition(0.275);
        rightStoneGrabber.setPosition(.8);
    }
    private void foundationUpGrabberDown(){
        foundationServo.setPosition(.62);   // originally .75
        rightStoneGrabber.setPosition(.35);
    }
    private void foundationDownGrabberDown(){
        foundationServo.setPosition(0.275);
        rightStoneGrabber.setPosition(.35);
    }
    private void foundationAndStoneAllIn(){
        foundationServo.setPosition(.3);
        rightStoneGrabber.setPosition(.7);
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
        if(skystoneLoc!= "right"){
            foundationServo.setPosition(0.3);
            rightStoneGrabber.setPosition(.8);}
        else {
            foundationServoRight.setPosition(0.95);
            grabberLeft.setPosition(.3);
        }
    }
    private void foundationUpGrabberDown2(){
        if(skystoneLoc!= "right"){
            foundationServo.setPosition(.62);   // originally .75
            rightStoneGrabber.setPosition(.35);}
        else{
            foundationServoRight.setPosition(.63);  // originally .6
            grabberLeft.setPosition(.7);
        }
    }
    private void foundationDownGrabberDown2(){
        if(skystoneLoc!= "right"){
            foundationServo.setPosition(0.3);
            rightStoneGrabber.setPosition(.35);}
        else{
            foundationServoRight.setPosition(.95);
            grabberLeft.setPosition(.7);
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
        wrist.setPosition(0.15);
    }
    private void liftHeight (double stage){
        liftEx1.setTargetPosition((int) (((stage * 3.95) - 1) * LIFT_COUNTS_PER_INCH));
        liftEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        liftEx1.setPower(liftPower);
    }
}
