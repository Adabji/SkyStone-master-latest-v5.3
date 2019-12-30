package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

public class StrafeRIGHTAuto extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(5).build());
        }
    }
}
