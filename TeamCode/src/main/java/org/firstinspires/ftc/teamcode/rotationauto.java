package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Disabled
@Config
@Autonomous(name = "rotationauto", group = "Autonomous")
public class rotationauto extends LinearOpMode {
    //double target;
    double errorNoWrap;
    double errorWrap;
    double Kp = .002;
    double leftPow;
    double rightPow;
    double landingHeading = 0;
    private boolean reachedHeading=false;




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



        while (!opModeIsActive() && !isStopRequested()) {

        }

        if (opModeIsActive()) {
            rotate(90);
            moveForward(drive,10);
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

    private void rotate(double target) {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        while (opModeIsActive()) {
            errorNoWrap = drive.getExternalHeading() - Math.toRadians(target);
           /* if(errorNoWrap>0){
                errorWrap=errorNoWrap;
            }
            else{
                errorWrap=errorNoWrap;
            }*/
            if (Math.abs(errorWrap) > Math.toRadians(10) && opModeIsActive()) {
                leftPow = errorWrap * Kp;
                rightPow = -errorWrap * Kp;
                drive.setMotorPowers(rightPow, rightPow, leftPow, leftPow);
                telemetry.addData("ErrorWrapped", errorWrap);
                telemetry.addData("LeftPow", leftPow);
                telemetry.addData("rightPow", rightPow);
                telemetry.addData("Heading",drive.getExternalHeading());
                telemetry.update();
            }
        }
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
