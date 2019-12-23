package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous(name = "AUTO Blue Building Site")
public class AutoREDBS extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(13, 47.25, 0)).build());

        // foundation servo down

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-22.75, 22.75, 90)).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().back(23).build());

        // foundation servo up

        drive.followTrajectorySync(drive.trajectoryBuilder().forward(40.75).build());
        drive.turnSync(Math.toRadians(90));
    }
}
