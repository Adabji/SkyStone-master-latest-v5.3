package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LiftTest", group = "TeleOp")
public class LiftTest extends OpMode {
    public static double AMPLITUDE = 10;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.5;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        telemetry.addData("x", AMPLITUDE * Math.sin(2 * Math.PI * FREQUENCY * getRuntime() + Math.toRadians(PHASE)));
        telemetry.update();
    }
}
