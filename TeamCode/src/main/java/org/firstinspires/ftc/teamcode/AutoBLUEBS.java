package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous(name = "NEW AUTO Blue Building Site")
public class AutoBLUEBS extends LinearOpMode {
    private DcMotorEx intakeMotor1, intakeMotor2;
    private DcMotorEx liftMotor1;
    private Servo foundationServoLeft, foundationServoRight;

    public void runOpMode() throws InterruptedException {
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake motor 1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake motor 2");

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "lift motor 1");

        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");
        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        foundationServosUp();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Ready. Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            moveBackward(drive, 40);
            strafeRight(drive, 10);
            moveBackward(drive, 2);

            // splineBotTo(drive, 10, -40, 0);

            foundationServosDown();

            // splineBotTo(drive, 22.75, 22.75, 90);
            moveForward(drive, 10);
            rotate(drive, 90);
            moveBackward(drive, 10);

            foundationServosUp();

            moveForward(drive, 40.75);
            rotate(drive, 90);
            moveBackward(drive, 15);
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
        foundationServoLeft.setPosition(0.23);
        foundationServoRight.setPosition(0.91);
    }

    private void foundationServosUp() {
        foundationServoLeft.setPosition(0.76);
        foundationServoRight.setPosition(0.38);
    }
}