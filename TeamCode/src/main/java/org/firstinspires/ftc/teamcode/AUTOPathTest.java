package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "AUTOPathTest", group = "Autonomous")
public class AUTOPathTest extends LinearOpMode {
    public static double initX = 0;
    public static double initY = 0;
    public static double stoneX = -8;
    public static double stoneY = -37;
    public static double lineX = 6.5;
    public static double lineY = 34;
    public static double foundationX = -12.5;
    public static double foundationY = 120;
    public static double line2X = 12;
    public static double line2Y = -78;
    public static double stone2X = 6.5;
    public static double stone2Y = -146;
    public static double line3X = 6;
    public static double line3Y = 72;
    public static double foundation2X = -13;
    public static double foundation2Y = 168;
    public static double strafeToPark = 16;
    public static double moveToPark = 35;
    public static double initHeading = 0;
    public static double turnAngle = Math.toRadians(92);
    public static double foundationHeading = 0;
    public static double foundationDown = 0.45;
    public static double foundationUp = 0.75;
    public static double grabberDown = 0.4;
    public static double grabberUp = 1;

    private Servo foundationServo, foundationServoRight, rightStoneGrabber;

    public DriveConstraints constraints = new DriveConstraints(
            60.0, 40.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        foundationServo = hardwareMap.servo.get("foundationServoLeft");
        foundationServoRight = hardwareMap.servo.get("foundationServoRight");
        rightStoneGrabber = hardwareMap.servo.get("rightStoneGrabber");

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            foundationServo.setPosition(foundationDown);
            rightStoneGrabber.setPosition(grabberUp);

            TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(new Pose2d(initX, initY, initHeading), constraints);

            // trajectoryBuilder.splineTo(new Pose2d(finalX, finalY, finalHeading));

            trajectoryBuilder.lineTo(new Vector2d(stoneX, stoneY), new LinearInterpolator(initHeading, turnAngle));

            drive.followTrajectorySync(trajectoryBuilder.build());

            rightStoneGrabber.setPosition(grabberDown);
            sleep(500);
            foundationServo.setPosition(foundationUp);

            // trajectoryBuilder.strafeTo(new Vector2d(lineX, lineY));

            drive.setPoseEstimate(new Pose2d(0, 0, 0));

            trajectoryBuilder = new TrajectoryBuilder(new Pose2d(initX, initY, initHeading), constraints);

            trajectoryBuilder.strafeTo(new Vector2d(lineX, lineY))
                    .strafeTo(new Vector2d(foundationX, foundationY));

            // drive.followTrajectorySync(trajectoryBuilder.build());

            drive.followTrajectorySync(trajectoryBuilder.build());

            foundationServo.setPosition(0.6);
            rightStoneGrabber.setPosition(grabberUp);

            sleep(200);

            foundationServo.setPosition(0.9);
            rightStoneGrabber.setPosition(grabberUp);

            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            sleep(100);
                trajectoryBuilder = new TrajectoryBuilder(new Pose2d(initX, initY, initHeading), constraints);
            trajectoryBuilder.strafeTo(new Vector2d(line2X, line2Y))
                    .strafeTo(new Vector2d(stone2X, stone2Y));

            drive.followTrajectorySync(trajectoryBuilder.build());

            foundationServo.setPosition(foundationDown);
            rightStoneGrabber.setPosition(grabberUp);

            moveBackward(drive, 3);

            rightStoneGrabber.setPosition(grabberDown);
            sleep(500);
            foundationServo.setPosition(foundationUp);

            // moveForward(drive, 3);

            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            sleep(100);
            trajectoryBuilder = new TrajectoryBuilder(new Pose2d(initX, initY, initHeading), constraints);
            trajectoryBuilder.strafeTo(new Vector2d(line3X, line3Y))
                    .strafeTo(new Vector2d(foundation2X, foundation2Y));

            drive.followTrajectorySync(trajectoryBuilder.build());

            foundationServo.setPosition(0.6);
            rightStoneGrabber.setPosition(grabberUp);

            sleep(300);

            strafeRight(drive, 10);
            moveBackward(drive, 1);
            rightStoneGrabber.setPosition(grabberUp);
            foundationServo.setPosition(foundationDown);
            foundationServoRight.setPosition(0.95);

            while(drive.getExternalHeading() > foundationHeading) {
                drive.setMotorPowers(0.5, 0.5, 0, 0);
            }

            drive.setMotorPowers(0, 0, 0, 0);
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            moveBackward(drive, 5);

            foundationServo.setPosition(foundationUp);
            foundationServoRight.setPosition(0.37);
            strafeRight(drive, strafeToPark);
            moveForward(drive, moveToPark);
            rotate(drive, -90);
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
}
