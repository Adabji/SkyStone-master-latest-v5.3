/**
 * Only lift test
 */

package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.*;

import java.util.Locale;
import java.lang.Math;
import java.util.Arrays;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.*;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LIFT TEST NEW", group = "TeleOp")
public class LiftTestNew extends OpMode {
    private static DcMotor liftMotor1;
    private static ExpansionHubMotor liftRE2;
    private static DcMotorEx liftEx1;
    private TouchSensor liftTouch;

    // drive train encoders
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.937;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);   // 43.4667

    // lift encoders
    static final double LIFT_COUNTS_PER_MOTOR_REV = 537.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = 1.0;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);  // 136.90275

    // lift things
    int liftstage = 0;
    int targetPos = 0;
    PIDFCoefficients pidfCoefficients;

    public static double lkp = 0;
    public static double lki = 0;
    public static double lkd = 0;
    public static double lkf = 0;
    public static double liftCurrentPos;

    boolean previousGP2LBPos = false;
    boolean previousGP2RBPos = false;

    boolean currentGP2LBPos;
    boolean currentGP2RBPos;

    @Override
    public void init() {
        liftMotor1 = hardwareMap.dcMotor.get("lift1");
        liftRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift1");
        liftEx1 = (DcMotorEx) hardwareMap.dcMotor.get("lift1");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        pidfCoefficients = new PIDFCoefficients(lkp, lki, lkd, lkf);

        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }

    @Override
    public void loop() {
        /**
         * UPDATE FIRMWARE
         */
        currentGP2LBPos = gamepad2.left_bumper;
        currentGP2RBPos = gamepad2.right_bumper;

        // lift - up a stage
        if (currentGP2RBPos && !previousGP2RBPos) {
            if (liftstage != 6) {
                liftstage++;
                targetPos = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
            }

            previousGP2RBPos = currentGP2RBPos;
        } else {
            previousGP2RBPos = currentGP2RBPos;
        }

        //  lift - down a stage
        if (currentGP2LBPos && !previousGP2LBPos) {
            if (liftstage != 0) {
                liftstage--;
                targetPos = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
            }

            previousGP2LBPos = currentGP2LBPos;
        } else {
            previousGP2LBPos = currentGP2LBPos;
        }

        // LIFT DcMotorEx
        // ---ATTEMPT #4 AFTER SCRIMMAGE---
        if (liftstage == 0 && liftTouch.isPressed()) {
            liftEx1.setPower(0);
        } else if (liftstage == 0 && !liftTouch.isPressed()) {
            liftEx1.setPower(-0.2);
        } else if (liftstage != 0 && liftEx1.getCurrentPosition() != targetPos) {
            liftEx1.setTargetPositionTolerance(targetPos);
            liftEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftEx1.setPower(0.5);
        }

        liftCurrentPos = liftMotor1.getCurrentPosition();

        telemetry.addData("liftTouchSensor", liftTouch.isPressed());
        telemetry.addData("lift1 encoder count", liftMotor1.getCurrentPosition());
        telemetry.addData("lift1 current", liftRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("Lift stage", liftstage);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("Lift PIDFCoefficients", liftEx1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("lkp", lkp);
        telemetry.addData("lki", lki);
        telemetry.addData("lkd", lkd);
        telemetry.addData("lkf", lkf);
        telemetry.addData("Current Lift Position", liftCurrentPos);

        telemetry.update();
    }
}