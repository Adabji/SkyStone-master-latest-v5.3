package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.revextensions2.ExpansionHubMotor;

@Disabled
@Autonomous(name = "BLUEAllAroundAuto", group = "Autonomous")
public class BLUEAllAroundAuto extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    private DcMotorEx intakeMotor1, intakeMotor2;
    private static DcMotorEx liftEx1;
    private Servo foundationServoLeft, foundationServoRight;

    // lift encoders
    static final double LIFT_COUNTS_PER_MOTOR_REV = 537.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = 1.0;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);  // 136.90275

    // Camera stuff
    String skystoneLoc = "";


    // hardware stuff
    private static Servo liftHoExt, wrist, grip;
    private TouchSensor liftTouch;
    double                  power = .30, correction, rotation;
    double                  rotationPower = 0.5;
    double                  movePower = 0.7;
    Orientation lastAngles = new Orientation();
    private DistanceSensor colorSensorDistance;

    // PIDF stuff
    PIDFCoefficients pidfCoefficients;
    double lkp = 6;
    double lki = 0;
    double lkd = 0;
    double lkf = 0;

    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake motor 1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake motor 2");

        liftEx1 = hardwareMap.get(DcMotorEx.class, "lift motor 1");

        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");
        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        foundationServosUp();

        /**
         * CHANGE DIRECTION
         * CHANGE DIRECTION
         * CHANGE DIRECTION
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
            moveForward(drive, 5);

            // FIRST SKYSTONE - OUTER ONE
            if (skystoneLoc.equals("right")) {
                strafeRight(drive, 15);
            } else if (skystoneLoc.equals("center")) {
                strafeRight(drive, 5);
            }

            moveForward(drive, 40);
            rotate(drive, 45);
            startIntakeMotors();
            moveForward(drive, 8);
            stopIntakeMotors();
            moveBackward(drive, 8);
            rotate(drive, 45);
            strafeLeft(drive, 25);
            moveBackward(drive, 80);
        }
    }

    private void startIntakeMotors() {
        intakeMotor1.setPower(-1);
        intakeMotor2.setPower(-1);
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
