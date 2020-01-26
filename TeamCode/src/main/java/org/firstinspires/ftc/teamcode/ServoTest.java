package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;

@Disabled
@Config
@Autonomous(name="ServoTEst", group = "Autonomous")
public class ServoTest extends LinearOpMode {
    private Servo foundationServo, backGrabber;
    public static double foundationDown = 0.45;
    public static double foundationUp = 0.75;
    public static double grabberDown = 0.4;
    public static double grabberUp = 1;

    @Override
    public void runOpMode() {
        foundationServo = hardwareMap.servo.get("foundationServoLeft");
        backGrabber = hardwareMap.servo.get("rightStoneGrabber");

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Waiting for start command");
        }

        while (opModeIsActive()) {
            foundationServo.setPosition(foundationDown);
            backGrabber.setPosition(grabberDown);
        }
    }
}
