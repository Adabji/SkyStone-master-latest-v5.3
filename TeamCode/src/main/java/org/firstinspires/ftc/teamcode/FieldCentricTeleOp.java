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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric TeleOp", group = "TeleOp")

@Config
public class FieldCentricTeleOp extends OpMode {
    private static double JoyStickAngleRad, JoyStickAngleDeg;
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, intakeMotor, liftMotor1, liftMotor2, liftMotor3;
    private static ExpansionHubMotor intakeMotorRE2, lift1RE2, lift2RE2, lift3RE2;
    private static DcMotorEx liftEx1, liftEx2, liftEx3;
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
            (WHEEL_DIAMETER_INCHES * 3.1415);


    static final double LIFT_COUNTS_PER_MOTOR_REV = 103.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = 1.0;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);  // 26.38

    // lift things
    int liftstage = 0;
    int targetPos = 0;
    double lPower = 0;
    double fightGPower = -0.226;
    PIDController pidLiftPower;
    PIDFCoefficients pidfCoefficients;
    double correction = 0;
    double liftPower = 1;
    double pidLPower = 1;
    int depositStage = 0;

    boolean fourBarIn = true;
    boolean aIsPressed = false;

    double position = 0.25;

    double lkp = 0.8;
    double lki = 0;
    double lkd = 0;
    double lkf = 0;

    boolean gp2RightBumperPressed = false;
    boolean gp2LeftBumperPressed = false;

    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get("left front");
        leftBackWheel = hardwareMap.dcMotor.get("left back");
        rightFrontWheel = hardwareMap.dcMotor.get("right front");
        rightBackWheel = hardwareMap.dcMotor.get("right back");

        intakeMotor = hardwareMap.dcMotor.get("intake motor");
        intakeMotorRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake motor");

        liftMotor1 = hardwareMap.dcMotor.get("lift1");
        liftMotor2 = hardwareMap.dcMotor.get("lift2");
        liftMotor3 = hardwareMap.dcMotor.get("lift3");
        lift1RE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift1");
        lift2RE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift2");
        lift3RE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift3");
        liftEx1 = (DcMotorEx) hardwareMap.dcMotor.get("lift1");
        liftEx2 = (DcMotorEx) hardwareMap.dcMotor.get("lift2");
        liftEx3 = (DcMotorEx) hardwareMap.dcMotor.get("lift3");

        pidfCoefficients = new PIDFCoefficients(lkp, lki, lkd, lkf);
        liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        liftEx2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        liftEx3.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");
        // hardwareMap.touchSensor.get("intakeTouch");

        leftElbow = hardwareMap.servo.get("leftv4b");
        rightElbow = hardwareMap.servo.get("rightv4b");
        wrist = hardwareMap.servo.get("rotategrabber");
        grip = hardwareMap.servo.get("grabber");
        foundationServoLeft = hardwareMap.servo.get("foundationServoLeft");
        foundationServoRight = hardwareMap.servo.get("foundationServoRight");

        // pidLiftPower = new PIDController(0.8, 0, 0);

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        // setLiftMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // setLiftMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setLExMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setLExMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        // automating block to move out of the robot after touch sensor is pressed
        try {
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
        }

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

        // Kill lift power (temp fix bc current issues)
        if(liftTouch.isPressed() && fourBarIn && liftstage == 0){
            setLiftMotorPower(0);
        }

        // ---BRING THIS BACK AGAIN---
        /* normal elbow out position for capstone
        if (gamepad2.right_bumper) {
            leftElbow.setPosition(0.17);
            rightElbow.setPosition(0.83);
            fourBarIn = false;
        }*/

        // lift - up a stage
        try {
            if (gamepad2.right_bumper) {
                if (liftstage != 6) {
                    liftstage++;
                    targetPos = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
                    Thread.sleep(100);
                }
            }

            //  lift - down a stage
            if (gamepad2.left_bumper) {
                if (liftstage != 0) {
                    liftstage--;
                    targetPos = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
                    Thread.sleep(100);
                }
            }
        } catch (InterruptedException ie) {
            telemetry.addLine("Lift thread interrupted. Exiting...");
            telemetry.update();
        }

        // ---ATTEMPT #1 AFTER SCRIMMAGE
        /*if (liftstage == 0 && !liftTouch.isPressed()) {
            setLiftMotorPower(-0.1);
        } else if (liftstage == 0 && liftTouch.isPressed()) {
            setLiftMotorPower(0);
        } else if (liftstage != 0 && liftMotor1.getCurrentPosition() < targetPos) {
            liftMotor1.setTargetPosition(targetPos);
            setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            setLiftMotorPower(0.8);
            while (liftMotor1.isBusy()) { }
        } else if (liftstage != 0 && liftMotor1.getCurrentPosition() > targetPos) {
            liftMotor1.setTargetPosition(targetPos);
            setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            setLiftMotorPower(-0.1);
            while (liftMotor1.isBusy()) { }
        }*/

        // ---ATTEMPT #2 AFTER SCRIMMAGE---
        /*if (liftstage == 0 && liftTouch.isPressed()) {
            setLiftMotorPower(0);
        } else if (liftstage == 0 && !liftTouch.isPressed()) {
            setLiftMotorPower(-0.2);
        } else if (liftstage != 0 && ((liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition() + liftMotor3.getCurrentPosition())/3) - targetPos > 100) {
            setLiftMotorPower(-0.1);
        } else if (liftstage != 0 && ((liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition() + liftMotor3.getCurrentPosition())/3) < targetPos) {
            setLiftMotorPower(0.5);
        } else if (liftstage != 0 && ((liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition() + liftMotor3.getCurrentPosition())/3) > targetPos) {
            setLiftMotorPower(0);
            // applyLiftBrakes();
        }*/

        // ---ATTEMPT #3 AFTER SCRIMMAGE---
        /*pidLiftPower.setSetpoint(targetPos);
        pidLiftPower.setInputRange(0, 690);
        pidLiftPower.setOutputRange(-liftPower, liftPower);
        pidLiftPower.enable();

        if (liftstage == 0 && liftTouch.isPressed()) {
            setLiftMotorPower(0);
        } else if (liftstage == 0 && !liftTouch.isPressed()) {
            setLiftMotorPower(-0.2);
        } else if (liftstage != 0 && (liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition() + liftMotor3.getCurrentPosition()) / 3 != targetPos) {
            pidLPower = pidLiftPower.performPID((liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition() + liftMotor3.getCurrentPosition()) / 3);
            // lTestPower = pidLPower;
            setMotorTargetPosition(targetPos);
            setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            setLiftMotorPower(pidLPower);
        }*/

        // ---ATTEMPT #4 AFTER SCRIMMAGE---
        if (liftstage == 0 && liftTouch.isPressed()) {
            setLExPower(0);
        } else if (liftstage == 0 && !liftTouch.isPressed()) {
            setLExPower(-0.2);
        } else if (liftstage != 0 && (liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition() + liftMotor3.getCurrentPosition()) / 3 != targetPos) {
            // pidLPower = pidLiftPower.performPID((liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition() + liftMotor3.getCurrentPosition()) / 3);
            // lTestPower = pidLPower;
            setLExTargetPos(targetPos);
            setLExMode(DcMotor.RunMode.RUN_TO_POSITION);
            setLExPower(0.5);
        }

        // lift PID things
        // even older lift things
        /*targetPos = (liftstage * 4) * COUNTS_PER_INCH;
        if (liftstage != 0 && liftMotor1.getCurrentPosition() != targetPos) {
            if (liftMotor1.getCurrentPosition() < targetPos) {
                goalStage = liftstage--;
                correction = pidLiftPower.performPID(liftMotor1.getCurrentPosition());
                setMotorTargetPosition((int)(((goalStage - liftstage) * 4) * COUNTS_PER_INCH) + (int)correction);
                setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(-0.5);
            } else if (liftMotor1.getCurrentPosition() > targetPos) {
                goalStage = liftstage++;
                correction = pidLiftPower.performPID(liftMotor1.getCurrentPosition());
                setMotorTargetPosition((int)(((goalStage - liftstage) * 4) * COUNTS_PER_INCH) + (int)correction);
                setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(0.5);
            }
        }*/

        // targetPos = (liftstage * 4) * COUNTS_PER_INCH;
        // old lift algorithm
        /*if (liftstage != 0 && liftMotor1.getCurrentPosition() != targetPos) {
            if (/*liftMotor1.getCurrentPosition() < targetPos targetPos - liftMotor1.getCurrentPosition() > 5) {
                // correction = pidLiftPower.performPID(liftMotor1.getCurrentPosition());
                // setMotorTargetPosition((int)((liftstage * 4) * COUNTS_PER_INCH) + (int)correction);
                setMotorTargetPosition((int)targetPos);
                setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(-0.5);
            } else if (/*liftMotor1.getCurrentPosition() > targetPos targetPos - liftMotor1.getCurrentPosition() < -5) {
                // correction = pidLiftPower.performPID(liftMotor1.getCurrentPosition());
                // setMotorTargetPosition((int)(-(liftstage * 4) * COUNTS_PER_INCH) + (int)correction);
                setMotorTargetPosition(-(int)targetPos);
                setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(.5);
            }
        }*/

        // new lift code
        /*targetPos = (liftstage * 4) * COUNTS_PER_INCH;
        // if (liftstage != 0 && liftMotor1.getCurrentPosition() != targetPos) {
            if (targetPos - liftMotor1.getCurrentPosition() > 5) {
                correction = pidLiftPower.performPID(liftMotor1.getPower());
                // setMotorTargetPosition((int)((liftstage * 4) * COUNTS_PER_INCH) + (int)correction);
                liftMotor1.setTargetPosition((int)targetPos);   // setMotorTargetPosition((int)targetPos);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);    // setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(1 + correction);
            } else if (targetPos - liftMotor1.getCurrentPosition() < -5) {
                correction = pidLiftPower.performPID(liftMotor1.getPower());
                // setMotorTargetPosition((int)(-(liftstage * 4) * COUNTS_PER_INCH) + (int)correction);
                liftMotor1.setTargetPosition((int)targetPos);  // setMotorTargetPosition((int)targetPos);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);    // setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(-1 + correction);
            }*/
        // }

        // lift power without encoders - whatever
        // correction = pidLiftPower.performPID(liftMotor1.getCurrentPosition());
        /*if (liftstage != 0 && liftMotor1.getCurrentPosition() != targetPos) {
            if (targetPos - liftMotor1.getCurrentPosition() > 5) {
                // correction = pidLiftPower.performPID(liftMotor1.getPower());
                // setMotorTargetPosition((int)((liftstage * 4) * COUNTS_PER_INCH) + (int)correction);
                // liftMotor1.setTargetPosition((int) targetPos);   // setMotorTargetPosition((int)targetPos);
                // liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);    // setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(1);
            } else if (targetPos - liftMotor1.getCurrentPosition() < -5) {
                correction = pidLiftPower.performPID(liftMotor1.getPower());
                // setMotorTargetPosition((int)(-(liftstage * 4) * COUNTS_PER_INCH) + (int)correction);
                // liftMotor1.setTargetPosition((int) targetPos);  // setMotorTargetPosition((int)targetPos);
                // liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);    // setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(-1);
            } else {
                setLiftMotorPower(0);
                applyBrakes();
            }
        }*/


        // ---LIFT PID---
        /*pidLiftPower.setSetpoint(targetPos);
        pidLiftPower.setInputRange(0, 690);
        pidLiftPower.setOutputRange(-liftPower, liftPower);
        pidLiftPower.enable();

        // liftPower = pidLiftPower.performPID(liftMotor1.getCurrentPosition());
        // setLiftMotorPower(liftPower);

        if (liftstage != 0 && liftMotor1.getCurrentPosition() != targetPos /*Math.abs(targetPos - liftMotor1.getCurrentPosition()) > 5) {
            pidLPower = pidLiftPower.performPID(liftMotor1.getCurrentPosition());
            // lTestPower = pidLPower;
            setMotorTargetPosition(targetPos);
            setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            setLiftMotorPower(pidLPower);
        } else if (liftstage == 0 && liftMotor1.getCurrentPosition() > 0) {
            setMotorTargetPosition(0);
            setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            setLiftMotorPower(-0.2);
        } else {
            setLiftMotorPower(0);
            applyBrakes();
        }*/

        /*if (liftstage != 0 && liftMotor1.getCurrentPosition() != targetPos) {
            if (liftMotor1.getCurrentPosition() < targetPos) {
                correction = pidLiftPower.performPID(liftMotor1.getPower());
                lmotor1poower = liftMotor1.getPower();
                // setLiftMotorPower(1 + correction);
                if (correction > 0) {
                    setLiftMotorPower(1 - correction);
                } else {
                    setLiftMotorPower(1 + correction);
                }
            } else if (liftMotor1.getCurrentPosition() > targetPos) {
                // correction = pidLiftPower.performPID(liftMotor1.getPower());
                setLiftMotorPower(0);
            } else {
                setLiftMotorPower(0);
                // applyBrakes();
            }
        } else {
            setLiftMotorPower(0);
            // applyBrakes();
        }*/


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

        // increase lift power by 0.02
        if (gamepad2.dpad_right) {
            if (lPower != 1) {
                lPower += 0.02;
                setLiftMotorPower(lPower);
            }
        }

        // decrease lift power by 0.02
        if (gamepad2.dpad_left) {
            if (lPower != 0) {
                lPower -= 0.02;
                setLiftMotorPower(lPower);
            }
        }

        // telemetry.addData("Touch  Sensor", intakeTouch.isPressed());
        /*telemetry.addData("liftTouchSensor", liftTouch.isPressed());

        telemetry.addData("lift1 encoder count", liftMotor1.getCurrentPosition());
        telemetry.addData("lift2 encoder count", liftMotor2.getCurrentPosition());
        telemetry.addData("lift3 encoder count", liftMotor3.getCurrentPosition());

        telemetry.addData("lift1 current", lift1RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lift2 current", lift2RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lift3 current", lift3RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        // telemetry.addData("intake current", intakeMotorRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));


        // telemetry.addData("pidLPower", pidLPower);
        telemetry.addData("Liftstage", liftstage);
        // telemetry.addData("COUNTS_PER_INCH", COUNTS_PER_INCH);
        // telemetry.addData("liftPower", liftPower);
        telemetry.addData("targetPos", targetPos);
        // telemetry.addData("difference", targetPos - liftMotor1.getCurrentPosition());

        // telemetry.addData("Servo Position", position);
        // telemetry.addData("foundationServoLeft Position", foundationServoLeft.getPosition());
        // telemetry.addData("foundationServoRight Position", foundationServoRight.getPosition());

        telemetry.update();*/
    }


    private int calcTargetPos(double tPos) {
        while (liftMotor1.getCurrentPosition() != liftMotor1.getTargetPosition()) {
            if (liftMotor1.getCurrentPosition() < liftMotor1.getTargetPosition()) {
                return -((int) tPos - liftMotor1.getCurrentPosition());
            }
        }
        return 0;
    }

    private void setLExMode(DcMotor.RunMode m) {
        liftEx1.setMode(m);
        liftEx2.setMode(m);
        liftEx3.setMode(m);
    }

    private void setLExPower(double p) {
        liftEx1.setPower(p);
        liftEx2.setPower(p);
        liftEx3.setPower(p);
    }

    private void setLExTargetPos(int target) {
        liftEx1.setTargetPosition(target);
        liftEx2.setTargetPosition(target);
        liftEx3.setTargetPosition(target);
    }

    private void setLiftMotorMode(DcMotor.RunMode m) {
        liftMotor1.setMode(m);
        liftMotor2.setMode(m);
        liftMotor3.setMode(m);
    }

    private void setLiftMotorPower(double p) {
        liftMotor1.setPower(p);
        liftMotor2.setPower(p);
        liftMotor3.setPower(p);
    }

    private void setMotorTargetPosition(int target) {
        liftMotor1.setTargetPosition(target);
        liftMotor2.setTargetPosition(target);
        liftMotor3.setTargetPosition(target);
    }

    private void applyLiftBrakes() {
        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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