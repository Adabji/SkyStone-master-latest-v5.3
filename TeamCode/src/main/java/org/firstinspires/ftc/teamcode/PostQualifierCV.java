package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name = "PostQualifierCV", group = "Autonomous")
public class PostQualifierCV extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private TESTSkystoneDetector skyStoneDetector;

    // Camera stuff
    String skystoneLoc = "";

    @Override
    public void runOpMode() {
        // SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new TESTSkystoneDetector();
        // phoneCam.setPipeline(skyStoneDetector);
        // phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        while (!opModeIsActive() && !isStopRequested()) {
            if (skyStoneDetector.isDetected()) {
                if (skyStoneDetector.foundRectangle().x < 80) {
                    skystoneLoc = "left";
                } else if (skyStoneDetector.foundRectangle().x < 150 && skyStoneDetector.foundRectangle().x > 80) {
                    skystoneLoc = "center";
                } else {
                    skystoneLoc = "right";
                }
            }

            phoneCam.setPipeline(skyStoneDetector);
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            phoneCam.openCameraDevice();

            telemetry.addData("Skystone Location = " + skyStoneDetector.foundRectangle(), skystoneLoc);
            telemetry.addData("Status", "Waiting for start command...");telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition().x, skystoneLoc);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
        }

        if (opModeIsActive()) {
            // phoneCam.stopStreaming();
        }
    }
}