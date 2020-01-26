package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

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
@Autonomous(name = "PostQualifierCV", group = "Autonomous")
public class PostQualifierCV extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private TESTSkystoneDetector skyStoneDetector;
    private DcMotorEx intakeMotor1, intakeMotor2;
    private Servo foundationServoLeft, foundationServoRight;

    // Camera stuff
    String skystoneLoc = "";

    // dashboard stuff
    public static boolean foundation = true;
    public static int leftAndCenterBlockMargin = 80;
    public static double turnAngle = 45;
    public static double strafeDist = 2;
    public static double moveToStoneDist = 18;
    public static double centerStrafe = 5;
    public static double moveABitToStone = 0;
    public static double moveToIntakeStone = 9;
    public static double strafeABitDiagonal = 19;
    public static double screenYPos = 160;
    public static double intakeInPower = 1;
    public static double intakeOutPower = -1;
    public static double strafeToFoundationDist = 33;
    public static double moveBackToFoundationDist = 15;
    public static double moveToLine = 35;

    // Timers
    long intakeInTimer = -1;
    long detectionTimer = -1;

    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake motor 1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake motor 2");

        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");
        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");

        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo positions
        foundationServosUp();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        skyStoneDetector = new TESTSkystoneDetector();

        /*phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);*/

        while (!opModeIsActive() && !isStopRequested()) {
            /*if (skyStoneDetector.isDetected()) {
                if (skyStoneDetector.getScreenPosition().x < 50) {
                    skystoneLoc = "left";
                } else if (skyStoneDetector.getScreenPosition().x < 150 && skyStoneDetector.getScreenPosition().x > 50) {
                    skystoneLoc = "center";
                } else {
                    skystoneLoc = "right";
                }
            }

            telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition(), skystoneLoc);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();*/

            telemetry.addLine("Robot is ready. Waiting for start command...");
            telemetry.update();
        }

        // WHILE just for testing - change to if
        if (opModeIsActive()) {
            // phoneCam.stopStreaming();

            phoneCam.openCameraDevice();
            phoneCam.setPipeline(skyStoneDetector);
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            skyStoneDetector.setFoundToFalse();

            detectionTimer = System.currentTimeMillis();

            while (System.currentTimeMillis() - detectionTimer < 500) { }

            // if (!skyStoneDetector.isDetected()) {
                if (skyStoneDetector.getScreenPosition().x < 50) {
                    skystoneLoc = "left";
                    strafeDist = 15;
                    strafeToFoundationDist = 43;
                } else if (skyStoneDetector.getScreenPosition().x < 150 && skyStoneDetector.getScreenPosition().x > 50) {
                    skystoneLoc = "center";
                    strafeDist = 2;
                    strafeToFoundationDist = 33;
                } else {
                    skystoneLoc = "right";
                    strafeDist = 6;
                    strafeToFoundationDist = 30;
                }
            // }

            telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition(), skystoneLoc);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            // sleep(10000);

            // open intake motors
            intakeMotor1.setPower(1);
            intakeMotor2.setPower(-1);
            sleep(50);
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);

            // telemetry.addData("Current heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();

            moveForward(drive, moveToStoneDist + 5);

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
            rotate(drive, turnAngle);
            // strafeRight(drive, 15.5);
            telemetry.update();
            // moveForward(drive, moveABitToStone);
            strafeRight(drive, strafeABitDiagonal);
            startIntakeMotors(intakeInPower);

            moveForward(drive, moveToIntakeStone);
            sleep(500);
            intakeInTimer = System.currentTimeMillis();
            while (System.currentTimeMillis() - intakeInTimer < 500) { }
            stopIntakeMotors();

            rotate(drive, 90 - turnAngle);
            strafeLeft(drive, strafeABitDiagonal + 2);
            // moveBackward(drive, 5);
            telemetry.update();
            // strafeLeft(drive, strafeBackDist);
            moveBackward(drive, 50);

            rotate(drive, 90);
            telemetry.update();

            intakeMotor1.setPower(-1);
            intakeMotor2.setPower(-1);
            sleep(500);
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);

            strafeLeft(drive, strafeToFoundationDist);
            moveBackward(drive, moveBackToFoundationDist);
            sleep(300);
            foundationServosDown();
            sleep(300);
            moveForward(drive, 10);

            telemetry.addData("heading", drive.getExternalHeading());
            telemetry.update();
            while(drive.getExternalHeading() > 1.571) {
                drive.setMotorPowers(0.5, 0.5, 0, 0);
            }

            drive.setMotorPowers(0, 0, 0, 0);

            foundation = false;
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            moveBackward(drive, 15);
            foundationServosUp();
            strafeRight(drive, 16);
            moveForward(drive, 35);
            rotate(drive, -90);
        }
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