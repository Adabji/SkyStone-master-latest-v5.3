/**
 * Field centric tele op: front is always the field's front, not the robot's front
 * <p>
 * NOTES:
 * - One button for no automation
 * - Lower stone placement
 * - Button to make movement slower
 *
 */

package org.firstinspires.ftc.teamcode;

import android.app.WallpaperInfo;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.*;

import java.util.Locale;
import java.lang.Math;
import java.util.Arrays;

import java.util.*;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.finalAutoHeading;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    private static double JoyStickAngleRad, JoyStickAngleDeg, intakeDistance;
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, intakeMotor1, intakeMotor2, intakeMotor3, liftMotor1;
    private static ExpansionHubMotor intakeMotorRE2, liftRE2;
    private static DcMotorEx liftEx1;
    private static Servo liftHoExt, wrist, grabber, foundationServoLeft, foundationServoRight, stoneHolder, capstoneServo;
    private static Servo tapeMeasure;
    //public static ColorSensor intakeColor;
    private static double PosXAngPosY, PosXAngNegY, NegXAng, Theta, r, outputX, outputY, heading;
    double headingAdjAfterReset = 0;
    static double slowMoReductionFactor = 1;
    Double Deg = Math.PI;
    BNO055IMU imu;
    Orientation angles;
    private TouchSensor liftTouch;
    private DistanceSensor intakeColor;

    // LIFT encoders
    static final double LIFT_COUNTS_PER_MOTOR_REV = 537.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = .5;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);  // 136.90275

    // lift things
    int nextLiftStage = 1;
    private double currentLiftStage = 0;
    private double desiredLiftStage = 0;
    int targetPos = 0;
    boolean targetBufferEarly = false;
    boolean targetBufferLate = false;
    double gffConstant = .3;
    double gffConstant2 = .3;
    double buffer1 = 40;
    double buffer2 = 10;
    double liftError;
    PIDFCoefficients pidfCoefficientsUp, pidfCoefficientsDown;
    // double currentPos = liftEx1.getCurrentPosition();
    boolean fasterDown = false;
    boolean isLiftTouchPressed = true;

    // lift PIDF
    public static double lkpUp = 16;
    public static double lkiUp = 0;
    public static double lkdUp = 0;
    public static double lkfUp = 0;

    public static double lkpDown = 4;
    public static double lkiDown = 0;
    public static double lkdDown = 0;
    public static double lkfDown = 0;

    boolean previousGP2LBPos = false;
    boolean previousGP2RBPos = false;
    boolean previousGP2LTPos = false;
    boolean previousGP2RTPos = false;
    boolean previousGP1DLPos = false;
    boolean previousGP1DUPos = false;
    boolean hoExt = false;


    boolean currentGP2LBPos;
    boolean currentGP2RBPos;
    boolean currentGP2LTPos;
    boolean currentGP2RTPos;
    boolean currentGP1DLPos;
    boolean currentGP1DUPos;

    double liftPower = 1;

    long gamePad2ATimer = -1;
    long gamePad2BTimer = -1;
    long liftDownExtTimer = -1;
    long stoneDropTimer= -1;
    long stoneAfterDropTimer = -1;
    long liftInTimer = -1;
    long stoneInRobotTimer = -1;
    long capstoneTimer = -1;
    long intakeTimer = -1;
    boolean servoPositionsAfterStart = true;
    boolean isAutomationOn = true;
    boolean isRBPressed = false;
    boolean liftUp = false;

    public static double foundationRightDown = 1;
    public static double foundationLeftDown = .24;
    public static double foundationRightUp = .5;
    public static double foundationLeftUp = .78;
    public static double liftExtOut = .935;
    public static double liftExtIn = .575;

    double r0, autoReset;

    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get("left front");
        leftBackWheel = hardwareMap.dcMotor.get("left back");
        rightFrontWheel = hardwareMap.dcMotor.get("right front");
        rightBackWheel = hardwareMap.dcMotor.get("right back");

        intakeMotor1 = hardwareMap.dcMotor.get("intake motor 1");
        intakeMotor2 = hardwareMap.dcMotor.get("intake motor 2");
        intakeMotor3 = hardwareMap.dcMotor.get("intake motor 3");

        intakeColor = hardwareMap.get(DistanceSensor.class,"intakeColor");


        intakeMotorRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake motor 1");

        liftMotor1 = hardwareMap.dcMotor.get("lift motor 1");
        liftEx1 = hardwareMap.get(DcMotorEx.class, "lift motor 1");
        liftRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift motor 1");

        // FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = dashboard.getTelemetry();

        pidfCoefficientsUp = new PIDFCoefficients(lkpUp, lkiUp, lkdUp, lkfUp);
        pidfCoefficientsDown = new PIDFCoefficients(lkpDown, lkiDown, lkdDown, lkfDown);

        intakeColor = hardwareMap.get(DistanceSensor.class, "intakeColor");
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");

        liftHoExt = hardwareMap.servo.get("liftHoExt");
        wrist = hardwareMap.servo.get("liftGrabberRotater");
        grabber = hardwareMap.servo.get("liftGrabber");
        foundationServoLeft = hardwareMap.servo.get("foundationServoLeft");
        foundationServoRight = hardwareMap.servo.get("foundationServoRight");
        // stoneHolder = hardwareMap.servo.get("stoneHolder");
        capstoneServo = hardwareMap.servo.get("capstoneServo");
        tapeMeasure = hardwareMap.servo.get("tapeMeasure");

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor3.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        // headingAdjAfterReset = finalAutoHeading;
    }

    /*@Override
    public void init_loop() {
        r0 = Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y);
        if (gamepad2.left_stick_y < 0) {
            autoReset = -Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x);
        }

        if (gamepad2.left_stick_y >= 0) {
            autoReset = 2 * Math.PI - Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x);
        }
    }*/

    @Override
    public void loop() {
        intakeDistance = intakeColor.getDistance(DistanceUnit.CM);
        liftError = Math.abs((((currentLiftStage * 3.9) - 1) * LIFT_COUNTS_PER_INCH)-liftEx1.getCurrentPosition());
        // servo position settings initialization at start


        // AUTOMATION KILL SWITCH
        // isAutomationOn =  gamepad1.left_trigger < 0.5;

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

        outputX = -Math.cos(heading - Theta + headingAdjAfterReset) * r;
        outputY = Math.sin(heading - Theta + headingAdjAfterReset) * r;

        heading = Math.toRadians(getAbsoluteHeading());

        // reset heading
        if (gamepad1.dpad_right) {
            // if (getAbsoluteHeading() < 0) {
            //     headingAdjAfterReset = Math.toRadians(getAbsoluteHeading());
            // } else {
                headingAdjAfterReset = -Math.toRadians(getAbsoluteHeading());
            // }
        }

        // slow-mo
        if (gamepad1.right_trigger > 0.5) {
            slowMoReductionFactor = 2;
        } else {
            slowMoReductionFactor = 1;
        }

        // intake out
        if (gamepad1.y) {
            intakeMotor1.setPower(-1);
            intakeMotor2.setPower(-1);
            intakeMotor3.setPower(-1);
        }

        // intake stop
        if (gamepad1.b || gamepad1.x) {
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);
            intakeMotor3.setPower(0);
        }

        // intake in
        if (gamepad1.a) {
            intakeMotor1.setPower(1);
            intakeMotor2.setPower(1);
            intakeMotor3.setPower(1);
        }

        // grabber - DEPOSIT STONE - then not grabbing - doesn't happen when the current lift stage is equal to 0
        if (currentLiftStage > 0) {
            if (gamepad1.dpad_down && stoneDropTimer == -1) {
                targetPos = (int) (((currentLiftStage * 4) - 2) * LIFT_COUNTS_PER_INCH);
                stoneDropTimer = System.currentTimeMillis();
            } else if (stoneDropTimer > 0 && System.currentTimeMillis() - stoneDropTimer > 1500 && stoneAfterDropTimer == -1) {
                grabber.setPosition(0.6);
                stoneDropTimer = -1;
                stoneAfterDropTimer = System.currentTimeMillis();
            } else if (stoneAfterDropTimer > 0 && System.currentTimeMillis() - stoneAfterDropTimer > 500) {
                targetPos = (int) ((currentLiftStage * 4) * LIFT_COUNTS_PER_INCH);
                stoneAfterDropTimer = -1;
            }
        }

        // To let go of the capstone
        if(gamepad1.left_trigger > 0.5) {
            capstoneServo.setPosition(0.25);
        } else {
            capstoneServo.setPosition(0.5);
        }

        // lift extension out position
        if (gamepad2.b && gamePad2BTimer == -1) {
            liftHoExt.setPosition(liftExtOut);
            gamePad2BTimer = System.currentTimeMillis();
            hoExt = true;
        } else if (gamePad2BTimer > 0 && System.currentTimeMillis() - gamePad2BTimer > 500) {
            gamePad2BTimer = -1;
        }

        // lift extension in
        if (gamepad2.a && gamePad2ATimer == -1) {
            gamePad2ATimer = System.currentTimeMillis();
        } else if (gamePad2ATimer > 0 && System.currentTimeMillis() - gamePad2ATimer > 300 && liftInTimer == -1) {
            liftHoExt.setPosition(liftExtIn);
            gamePad2ATimer = -1;
            hoExt = false;
            liftInTimer = System.currentTimeMillis();
        } else if (liftInTimer > 0 && System.currentTimeMillis() - liftInTimer > 700) {
            // grabber.setPosition(0.6);
            liftInTimer = -1;
        }
        /* if (intakeDistance > 6.5 && intakeDistance < 7.5 && isLiftTouchPressed && !hoExt){
            grabber.setPosition(0.44);
            wrist.setPosition(0.1);
        }
        /*else if (intakeDistance > 3.5 && intakeDistance < 5.5 && isLiftTouchPressed && !hoExt){

                grabber.setPosition(0.4);
                wrist.setPosition(0.5);
                intakeMotor1.setPower(0);
                intakeMotor2.setPower(0);
                intakeMotor3.setPower(0);

        }*/
        /*// grabber direction - out
        if (gamepad2.dpad_up) {
            wrist.setPosition(0.85);
        }

        // grabber direction - in
        if (gamepad2.dpad_down) {
            wrist.setPosition(0.18);
        }*/

        // grabber - grabbing
        if (gamepad2.x) {
            grabber.setPosition(0.4);
            wrist.setPosition(0.5);
        }

        // grabber - not grabbing
        if (gamepad2.y) {
            grabber.setPosition(0.8);
            wrist.setPosition(0.1);
        }

        // tape measure
        if (gamepad2.dpad_down) {
            tapeMeasure.setPosition(0.75);
        } else if (gamepad2.dpad_up) {
            tapeMeasure.setPosition(0.25);
        } else {
            tapeMeasure.setPosition(0.5);
        }
        if (gamepad2.dpad_right) {
            grabber.setPosition(0.44);
            wrist.setPosition(0.1);
        }

        /* // stone holder out - not holding
        if (gamepad2.dpad_left) {
            stoneHolder.setPosition(0.4);
        }

        // stone holder in - holding the stone in place
        if (gamepad2.dpad_right) {
            stoneHolder.setPosition(0);
        }*/

        // foundation servo - Up
        if (gamepad1.left_bumper) {
            foundationServoRight.setPosition(foundationRightUp);
            foundationServoLeft.setPosition(foundationLeftUp);
        }

        // foundation servo - Down
        if (gamepad1.right_bumper) {
            foundationServoRight.setPosition(foundationRightDown);
            foundationServoLeft.setPosition(foundationLeftDown);
        }

        currentGP1DLPos = gamepad1.dpad_left;
        currentGP1DUPos = gamepad1.dpad_up;

        // capstone servo down
       /* if (currentGP1DLPos && !previousGP1DLPos) {
            capstoneServo.setPosition(0.75);
            /*capstoneTimer = System.currentTimeMillis();
            while (capstoneTimer > 0 && System.currentTimeMillis() - capstoneTimer < 1000) { }
            capstoneServo.setPosition(0);
            capstoneTimer = -1;*/

          //  previousGP1DLPos = currentGP1DLPos;
        //} else {
         //   previousGP1DLPos = currentGP1DLPos;
        //}

        // capstone servo up
        /*if (currentGP1DUPos && !previousGP1DUPos) {
            capstoneServo.setPosition(0);
            previousGP1DUPos = currentGP1DUPos;
        } else {
            previousGP1DUPos = currentGP1DUPos;
        }*/

        currentGP2LBPos = gamepad2.left_bumper;
        currentGP2RBPos = gamepad2.right_bumper;
        currentGP2LTPos = gamepad2.left_trigger > 0.5;
        currentGP2RTPos = gamepad2.right_trigger > 0.5;

        // right bumper: goes up "desiredLiftStage" number of stages, sets desiredLiftStage to 1
        if (currentGP2RBPos && !previousGP2RBPos) {
            currentLiftStage += desiredLiftStage;
            if (currentLiftStage > 11) { currentLiftStage = 11; }
            desiredLiftStage = 1;
            isLiftTouchPressed = false;
            liftUp = false;

            liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficientsUp);

            previousGP2RBPos = currentGP2RBPos;
        } else {
            previousGP2RBPos = currentGP2RBPos;
        }

        // left bumper: lift completely down and desiredLiftStage = 0
        if (currentGP2LBPos && !previousGP2LBPos && currentLiftStage != 0) {
            desiredLiftStage = currentLiftStage + 2;
            currentLiftStage = 0;

            previousGP2LBPos = currentGP2LBPos;
        } else {
            previousGP2LBPos = currentGP2LBPos;
        }
        if (gamepad2.dpad_left){
            desiredLiftStage = 0;
        }

        // right trigger: nextLiftStage + 1
        if (currentGP2RTPos && !previousGP2RTPos) {
            desiredLiftStage++;

            previousGP2RTPos = currentGP2RTPos;
        } else {
            previousGP2RTPos = currentGP2RTPos;
        }

        // left trigger: lift down one stage but desiredLiftStage doesn't change
        if (currentGP2LTPos && !previousGP2LTPos) {
            if (currentLiftStage > 0) currentLiftStage--;
            liftUp = false;

            liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficientsDown);

            previousGP2LTPos = currentGP2LTPos;
        } else {
            previousGP2LTPos = currentGP2LTPos;
        }

        // LIFT DcMotorEx
        if (!isLiftTouchPressed && currentLiftStage == 0 && !liftTouch.isPressed()) {
            liftEx1.setTargetPosition(0);
            liftEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftEx1.setPower(-0.2);
        } else if (currentLiftStage == 0 && liftTouch.isPressed()) {
            isLiftTouchPressed = true;
            liftEx1.setPower(0);
            liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (!liftUp && !isLiftTouchPressed && currentLiftStage != 0 /*&& liftError>10*//*liftEx1.getCurrentPosition() != (int) (((currentLiftStage * 3.95) - 1) * LIFT_COUNTS_PER_INCH) && !targetBufferEarly && !targetBufferLate*/) {
            if (currentLiftStage == 11) {
                liftEx1.setTargetPosition((int) (((currentLiftStage * 3.95) - 3.75) * LIFT_COUNTS_PER_INCH));
            } else {
                liftEx1.setTargetPosition((int) (((currentLiftStage * 3.95) - 1) * LIFT_COUNTS_PER_INCH));
            }

            liftEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftEx1.setPower(liftPower);
            liftUp = true;
        }/* else {
            liftEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftEx1.setPower(.13);
        }*/

        /*
        // lift - 1 stage above the previous stage
        if (currentGP2RBPos && !previousGP2RBPos) {
            if (nextLiftStage <= 7) {
                currentLiftStage = nextLiftStage;
                targetPos = (int) ((currentLiftStage * 4) * LIFT_COUNTS_PER_INCH);
            }

            previousGP2RBPos = currentGP2RBPos;
        } else {
            previousGP2RBPos = currentGP2RBPos;
        }

        // lift - completely down
        if (currentGP2LBPos && !previousGP2LBPos) {
            if (currentLiftStage == 0) {
                return;
            }

            liftDownExtTimer = System.currentTimeMillis();
            wrist.setPosition(0.18);
            // grabber.setPosition(0.6);

            while (System.currentTimeMillis() - liftDownExtTimer < 300) { }

            liftHoExt.setPosition(0.45);
            liftDownExtTimer = -1;

            currentLiftStage = 0;
            targetPos = 0;
            if (nextLiftStage != 7) {
                nextLiftStage++;
            }

            previousGP2LBPos = currentGP2LBPos;
        } else {
            previousGP2LBPos = currentGP2LBPos;
        }

        // lift - up 1 stage
        if (currentGP2RTPos && !previousGP2RTPos) {
            if (nextLiftStage >= 0 && nextLiftStage < 7 && currentLiftStage != 7) {
                currentLiftStage++;
            }

            previousGP2RTPos = currentGP2RTPos;
            nextLiftStage = currentLiftStage;
            targetPos = (int) ((currentLiftStage * 4) * LIFT_COUNTS_PER_INCH);
        } else {
            previousGP2RTPos = currentGP2RTPos;
        }

        // lift - down 1 stage
        if (currentGP2LTPos && !previousGP2LTPos) {
            if (nextLiftStage > 0 && nextLiftStage <= 7 && currentLiftStage != 0) {
                currentLiftStage--;
            }

            previousGP2LTPos = currentGP2LTPos;
            nextLiftStage = currentLiftStage;
            targetPos = (int) ((currentLiftStage * 4) * LIFT_COUNTS_PER_INCH);
        } else {
            previousGP2LTPos = currentGP2LTPos;
        }

        // LIFT DcMotorEx
        if (currentLiftStage == 0 && !liftTouch.isPressed()) {
            liftEx1.setTargetPosition(targetPos);
            liftEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftEx1.setPower(-0.7);
        } else if (currentLiftStage == 0 && liftTouch.isPressed()) {
            liftEx1.setPower(0);
            liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (currentLiftStage != 0 && liftEx1.getCurrentPosition() != targetPos) {
            liftEx1.setTargetPosition(targetPos);
            liftEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
            liftEx1.setPower(liftPower);
        }*/
            /*else if (!isLiftTouchPressed && currentLiftStage != 0 && liftEx1.getCurrentPosition() != (int) (((currentLiftStage * 4) - 1) * LIFT_COUNTS_PER_INCH) && targetBufferEarly && targetBufferLate){
            liftEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftEx1.setPower(gffConstant);
        }*/
       /* else if (!isLiftTouchPressed && currentLiftStage != 0 && liftEx1.getCurrentPosition() != (int) (((currentLiftStage * 4) - 1) * LIFT_COUNTS_PER_INCH) && targetBufferEarly && targetBufferLate){
            liftEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftEx1.setPower(gffConstant2);
        }*/

        if (isLiftTouchPressed) {
            liftEx1.setPower(0);
            liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // linkage extends after it goes up
        /*if (isRBPressed) {
            if (currentLiftStage != 0 && Math.abs(liftEx1.getCurrentPosition() - targetPos) < 5) {
                liftHoExt.setPosition(1);
                isRBPressed = false;
            }
        }*/

        // telemetry.addData("lift1 encoder count", liftMotor1.getCurrentPosition());
        telemetry.addData("resetAdj", headingAdjAfterReset);
        telemetry.addData("lift1 current", liftRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lift1 power", liftEx1.getPower());
        telemetry.addData("currentLiftStage", currentLiftStage);
        telemetry.addData("desiredLiftStage", desiredLiftStage);
        telemetry.addData("isLiftTouchPressed", isLiftTouchPressed);
        telemetry.addData("HoExt", hoExt);

        // telemetry.addData("NextLiftstage", nextLiftStage);
        // telemetry.addData("targetPos", targetPos);
        // telemetry.addData("intakeDistanceSensor Reading", intakeColor.getDistance(DistanceUnit.CM));
        // telemetry.addData("liftTouchSensor", liftTouch.isPressed());
        // telemetry.addData("isAutomationOn", isAutomationOn);

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

        leftFrontWheel.setPower(leftFront/slowMoReductionFactor);
        rightFrontWheel.setPower(rightFront/slowMoReductionFactor);
        leftBackWheel.setPower(leftBack/slowMoReductionFactor);
        rightBackWheel.setPower(rightBack/slowMoReductionFactor);
    }
}