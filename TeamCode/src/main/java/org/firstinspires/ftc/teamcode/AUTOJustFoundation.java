package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.finalAutoHeading;

@Disabled
@Autonomous(name = "Just Foundation red", group="Autonomous")
public class AUTOJustFoundation extends LinearOpMode {

    private DcMotorEx intakeMotor1, intakeMotor2;
    private static DcMotorEx liftEx1;
    private Servo foundationServoLeft, foundationServoRight;

    // dashboard stuff
    public static double moveToLine = 35;

    public void runOpMode() {
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake motor 1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake motor 2");
        liftEx1 = hardwareMap.get(DcMotorEx.class, "lift motor 1");

        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");
        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");

        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Servo positions
        foundationServosUp();

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        if (opModeIsActive()) {
            moveBackward(drive, 45);
            strafeLeft(drive, 25);

            while(drive.getExternalHeading() > 0.785) {
                drive.setMotorPowers(0.5, 0.5, 0, 0);
            }
            // rotate(drive,-90);
            while(drive.getExternalHeading() > 1.571) {
                drive.setMotorPowers(0, 0, -0.5, -0.5);
            }

            drive.setMotorPowers(0, 0, 0, 0);

            foundationServosUp();
            moveForward(drive, moveToLine);
            rotate(drive, -90);

            finalAutoHeading = drive.getExternalHeading();
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

    private void foundationServosDown() {
        foundationServoLeft.setPosition(0.21);
        foundationServoRight.setPosition(0.95);
    }

    private void foundationServosUp() {
        foundationServoLeft.setPosition(0.77);
        foundationServoRight.setPosition(0.37);
    }
}
