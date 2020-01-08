package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous(name="Move Right Auto", group="Autonomous")
public class StrafeRIGHTAuto extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            drive.followTrajectorySync(drive.trajectoryBuilder().forward(2).build());
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(10).build());
            drive.followTrajectorySync(drive.trajectoryBuilder().back(2).build());
        }
    }
}
