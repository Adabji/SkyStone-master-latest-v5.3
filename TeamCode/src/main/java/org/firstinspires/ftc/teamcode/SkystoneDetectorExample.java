package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */

@Disabled
@Autonomous(name = "Skystone Detector OpMode", group="Autonomous")
public class SkystoneDetectorExample extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    // Camera stuff
    String skystoneLoc = "";

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new TESTSkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!opModeIsActive() && !isStopRequested()) {
            if (skyStoneDetector.isDetected()) {
                if (skyStoneDetector.getScreenPosition().x < 80) {
                    skystoneLoc = "left";
                } else if (skyStoneDetector.getScreenPosition().x < 150 && skyStoneDetector.getScreenPosition().x > 80) {
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
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition().x, skystoneLoc);
            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            // telemetry.addData("Frame Count", phoneCam.getFrameCount());
            // telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            // telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            // telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            // telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            // telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}