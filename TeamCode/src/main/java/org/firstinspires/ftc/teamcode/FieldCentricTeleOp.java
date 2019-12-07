/**
 * Field centric tele op: front is always the field's front, not the robot's front
 * <p>
 * NOTES:
 * - BRING BACK CAPSTONE POSITION COMMAND
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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    private static double JoyStickAngleRad, JoyStickAngleDeg;
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, intakeMotor, liftMotor1;
    private static ExpansionHubMotor intakeMotorRE2, liftRE2;
    private static DcMotorEx liftEx1;
    private static Servo leftElbow, rightElbow, wrist, grip, foundationServoLeft, foundationServoRight;
    private static double PosXAngPosY, PosXAngNegY, NegXAng, Theta, r, outputX, outputY, heading;
    Double Deg = Math.PI;
    BNO055IMU imu;
    Orientation angles;
    double zeroPos = 0;
    private TouchSensor liftTouch;
    private TouchSensor intakeTouch;

    // encoder things
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.937;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);   // 43.4667


    static final double LIFT_COUNTS_PER_MOTOR_REV = 537.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = 1.0;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);  // 136.90275

    // lift things
    int liftstage = 0;
    int targetPos = 0;
    double fightGPower = -0.226;
    PIDController pidLiftPower;
    PIDFCoefficients pidfCoefficients;

    boolean fourBarIn = true;
    boolean aIsPressed = false;

    double position = 0.25;

    double lkp = 0;
    double lki = 0;
    double lkd = 0;
    double lkf = 0;

    boolean previousGP2LBPos = false;
    boolean previousGP2RBPos = false;

    boolean currentGP2LBPos;
    boolean currentGP2RBPos;

    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get("left front");
        leftBackWheel = hardwareMap.dcMotor.get("left back");
        rightFrontWheel = hardwareMap.dcMotor.get("right front");
        rightBackWheel = hardwareMap.dcMotor.get("right back");

        intakeMotor = hardwareMap.dcMotor.get("intake motor");
        intakeMotorRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake motor");

        liftMotor1 = hardwareMap.dcMotor.get("lift1");
        liftRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift1");
        liftEx1 = (DcMotorEx) hardwareMap.dcMotor.get("lift1");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        pidfCoefficients = new PIDFCoefficients(lkp, lki, lkd, lkf);

        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");

        leftElbow = hardwareMap.servo.get("leftv4b");
        rightElbow = hardwareMap.servo.get("rightv4b");
        wrist = hardwareMap.servo.get("rotategrabber");
        grip = hardwareMap.servo.get("grabber");
        foundationServoLeft = hardwareMap.servo.get("foundationServoLeft");
        foundationServoRight = hardwareMap.servo.get("foundationServoRight");

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        foundationServoLeft.setPosition(0.2);
        foundationServoRight.setPosition(0.87);
        // leftElbow.setPosition(0.85);
        // rightElbow.setPosition(0.15);
        wrist.setPosition(0.6);
        grip.setPosition(0.49);
    }

    @Override
    public void loop() {
        double inputY = gamepad1.left_stick_y;
        double inputX = -gamepad1.left_stick_x;
        double inputC = -gamepad1.right_stick_x;
        // the negative signs in front of the gamepad inputs may need to be removed.
        driveMecanum(inputY, inputX, inputC);

        if (gamepad1.left_stick_x >= 0 && gamepad1.left_stick_y < 0) {
            JoyStickAngleRad = PosXAngPosY;
        } else if (gamepad1.left_stick_x >= 0 && gamepad1.left_stick_y >= 0) {
            JoyStickAngleRad = PosXAngNegY;
        } else {
            JoyStickAngleRad = NegXAng;
        }

        r = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);
        if (gamepad1.left_stick_y < 0) {
            Theta = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        }

        if (gamepad1.left_stick_y >= 0) {
            Theta = 2 * Math.PI - Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        }

        outputX = -Math.cos(heading - Theta) * r;
        outputY = Math.sin(heading - Theta) * r;
        /*telemetry.addData("LeftX",gamepad1.left_stick_x);
        telemetry.addData("LeftY", -gamepad1.left_stick_y);
        telemetry.addData("r",r);
        telemetry.addData("Theta", Math.toDegrees(Theta));
        telemetry.addData("outputX",outputX);
        telemetry.addData("outputY",outputY);
        telemetry.addData("angle:",adjustAngle(getAbsoluteHeading()));
        telemetry.addData("Angle:",imu.getPosition().z);
        telemetry.update();*/
        heading = Math.toRadians(getAbsoluteHeading());

        /*if (gamepad2.x){
            heading = 0;
        }*/

        // intake in
        if (gamepad1.a) {
            aIsPressed = true;
            intakeMotor.setPower(-1);
        }

        // intake stop
        if (gamepad1.b || gamepad1.x) {
            intakeMotor.setPower(0);
        }

        // intake out
        if (gamepad1.y) {
            aIsPressed = false;
            intakeMotor.setPower(1);
        }

        // automating block to move out of the robot after touch sensor is pressed - remove try catch
        /*try {
            if (intakeTouch.isPressed() && fourBarIn)  {
                fourBarIn = false;
                intakeMotor.setPower(0);
                grip.setPosition(0.55);
                Thread.sleep(500);
                leftElbow.setPosition(0.17);
                rightElbow.setPosition(0.83);
            }
        } catch (InterruptedException ie) {
            telemetry.addLine("Intake thread interrupted. Exiting...");
            telemetry.update();
        }*/

        // Lower elbow out position
        if (gamepad2.b) {
            leftElbow.setPosition(0.06);
            rightElbow.setPosition(0.94);
            fourBarIn = false;
        }

        // Elbow in
        if (gamepad2.a) {
            leftElbow.setPosition(0.9);
            rightElbow.setPosition(0.1);
            fourBarIn = true;
            wrist.setPosition(0.6);
            grip.setPosition(.45);
        }

        // grabber direction - skystone is horizontal
        if (gamepad2.dpad_up) {
            wrist.setPosition(0.22);
        }

        // grabber direction - skystone is vertical
        if (gamepad2.dpad_down) {
            wrist.setPosition(0.6);
        }

        // grabber - not grabbing
        if (gamepad2.y) {
            grip.setPosition(0.45);
        }

        // grabber - grabbing
        if (gamepad2.x) {
            grip.setPosition(0.55);
        }

        /* deposit stone
        if (gamepad1.right_trigger > 0.5 && liftstage != 0) {
            depositStage = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
            targetPos = depositStage - 53;  // 26.38 lift encoder clicks = 1 inch. Change this if necessary.
        } else if (gamepad1.right_trigger < 0.5 && liftstage != 0) {
            targetPos = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
        }*/

        // foundation servo - Up
        if (gamepad1.left_bumper) {
            foundationServoLeft.setPosition(0.41);
            foundationServoRight.setPosition(0.674);
        }

        // foundation servo - Down
        if (gamepad1.right_bumper) {
            foundationServoLeft.setPosition(0.2);
            foundationServoRight.setPosition(0.87);
        }

        // foundation servo - Up by 0.02
        if (gamepad1.dpad_right) {
            position = foundationServoLeft.getPosition() + 0.02;
            foundationServoLeft.setPosition(position);
            foundationServoRight.setPosition(1.025 - position);
        }

        // foundation servo - Down by 0.02
        if (gamepad1.dpad_left) {
            position = foundationServoLeft.getPosition() - 0.02;
            foundationServoLeft.setPosition(position);
            foundationServoRight.setPosition(1.025 - position);
        }

        // ---BRING THIS BACK AGAIN---
        /* elbow out position for capstone
        if (gamepad2.right_bumper) {
            leftElbow.setPosition(0.17);
            rightElbow.setPosition(0.83);
            fourBarIn = false;
        }*/

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

        // intake - counter stall
        /*try {
            if (aIsPressed && intakeMotorRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 7) {
                telemetry.addLine("Intake Motor stalling. Restarting motor...");
                telemetry.update();
                intakeMotor.setPower(0);
                Thread.sleep(200);
                intakeMotor.setPower(1);
                Thread.sleep(200);
                intakeMotor.setPower(-1);
            }
        } catch (InterruptedException ie) {
            telemetry.addLine("Thread interrupted. Exiting...");
            telemetry.update();
        }*/

        // telemetry.addData("Touch  Sensor", intakeTouch.isPressed());
        telemetry.addData("liftTouchSensor", liftTouch.isPressed());

        // telemetry.addData("lift1 encoder count", liftMotor1.getCurrentPosition());
        // telemetry.addData("lift1 current", liftRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        // telemetry.addData("intake current", intakeMotorRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("Liftstage", liftstage);
        telemetry.addData("targetPos", targetPos);
        // telemetry.addData("lift pidfCoefficients", liftEx1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        // telemetry.addData("Servo Position", position);
        // telemetry.addData("foundationServoLeft Position", foundationServoLeft.getPosition());
        // telemetry.addData("foundationServoRight Position", foundationServoRight.getPosition());

        telemetry.update();
    }

    public double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    private Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private static void driveMecanum(double forwards, double horizontal, double turning) {
        double leftFront = outputY + outputX + turning;
        double leftBack = outputY - outputX + turning;
        double rightFront = outputY - outputX - turning;
        double rightBack = outputY + outputX - turning;

        double[] wheelPowers = {Math.abs(rightFront), Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightBack)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];
        if (biggestInput > 1) {
            leftFront /= biggestInput;
            leftBack /= biggestInput;
            rightFront /= biggestInput;
            rightBack /= biggestInput;
        }

        leftFrontWheel.setPower(leftFront);
        rightFrontWheel.setPower(rightFront);
        leftBackWheel.setPower(leftBack);
        rightBackWheel.setPower(rightBack);
    }
}