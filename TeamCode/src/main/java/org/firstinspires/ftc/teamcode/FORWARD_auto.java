package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous(name="FORWARD Auto", group="Autonomous")
public class FORWARD_auto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            Thread.sleep(25000);
            drive.followTrajectorySync(drive.trajectoryBuilder().forward(25).build());
        }
    }
}
