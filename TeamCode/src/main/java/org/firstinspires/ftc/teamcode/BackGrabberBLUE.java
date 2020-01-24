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

@Config
@Autonomous(name = "BackGrabberBLUE", group = "Autonomous")
public class BackGrabberBLUE extends LinearOpMode {
    public static double movementa1 = 20;
    public static double movementb2 = 4;
    public static double movementc3 = 90;
    public static double movementd4 = 25;
    public static double movemente5 = 10;
    public static double movementf6 = 88;
    public static double movementg7 = 85;
    public static double movementh8 = -90;
    public static double movementi9 = 12;
  /*  public static double movementj10 = -92;
    public static double movementk11 = 16;
    public static double movementl12 = 90;
    public static double movementm13 = 12;
    public static double movementn14 = 11;
    public static double movemento15 = -90;
    public static double movementp16 = 15;*/
    public static double movementq17 = 4;
    public static double movementr18 = 10;
   // public static double movements19 = 30;
    //public static double movementt20 = 12;


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

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            strafeRight(drive, movementa1);
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
            moveForward(drive,3);
            grabFoundation();
            sleep(500);
            while(drive.getExternalHeading() < 3.1415) { drive.setMotorPowers(0, 0, 0.5, 0.5); }
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setPoseEstimate(0,0,0);
            releaseFoundation();
            //strafeRight(drive,movementr18);
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
}
