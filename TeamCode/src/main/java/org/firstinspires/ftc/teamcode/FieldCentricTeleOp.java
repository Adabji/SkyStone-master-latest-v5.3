/**
 * Field centric tele op: front is always the field's front, not the robot's front
 *
 * NOTES:
 * -
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.*;

import java.util.Locale;
import java.lang.Math;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    private static double JoyStickAngleRad, JoyStickAngleDeg;
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, intakeMotor, liftMotor1, liftMotor2, liftMotor3;
    private static ExpansionHubMotor intakeMotorRE2, lift1RE2, lift2RE2, lift3RE2;
    private static Servo leftElbow, rightElbow, wrist, grip;
    private static double PosXAngPosY, PosXAngNegY, NegXAng, Theta, r, outputX, outputY, heading;
    Double Deg = Math.PI;
    // BNO055IMU imu;
    // Orientation angles;
    double zeroPos = 0;

    // encoder things
    static final double     COUNTS_PER_MOTOR_REV  = 537.6;
    static final double     DRIVE_GEAR_REDUCTION  = 1.0;
    static final double     WHEEL_DIAMETER_INCHES = 3.937;
    static final double     COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // lift things
    int liftstage = 0;
    double lPower = 0;
    double fightGPower = -0.226;
    PIDController pidLiftDrive;
    double correction = 0;

    boolean aIsPressed = false;

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

        setLiftMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElbow = hardwareMap.servo.get("leftv4b");
        rightElbow = hardwareMap.servo.get("rightv4b");
        wrist = hardwareMap.servo.get("rotategrabber");
        grip = hardwareMap.servo.get("grabber");

        pidLiftDrive = new PIDController(.05, .00003, 0);

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);*/

        leftElbow.setPosition(0.75);
        rightElbow.setPosition(0.25);
        wrist.setPosition(0.6);
        grip.setPosition(0.45);
    }

    @Override
    public void loop() {
        // fight gravity to keep up in the air
        // setLiftMotorPower(fightGPower);

        double inputY = gamepad1.left_stick_y;
        double inputX = -gamepad1.left_stick_x;
        double inputC = -gamepad1.right_stick_x;
        // the negative signs in front of the gamepad inputs may need to be removed.
        driveMecanum(inputY, inputX, inputC);

        if(gamepad1.left_stick_x >=0 && gamepad1.left_stick_y < 0){
            JoyStickAngleRad = PosXAngPosY;
        }
        else if(gamepad1.left_stick_x >=0 && gamepad1.left_stick_y>=0){
            JoyStickAngleRad = PosXAngNegY;
        }
        else {
            JoyStickAngleRad = NegXAng;
        }
        r = Math.sqrt(gamepad1.left_stick_x* gamepad1.left_stick_x + gamepad1.left_stick_y* gamepad1.left_stick_y);
        if( gamepad1.left_stick_y < 0){ Theta = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);}
        if( gamepad1.left_stick_y >= 0){ Theta = 2*Math.PI - Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);}
        outputX = -Math.cos(heading - Theta)*r;
        outputY = Math.sin(heading - Theta)*r;
        /*telemetry.addData("LeftX",gamepad1.left_stick_x);
        telemetry.addData("LeftY", -gamepad1.left_stick_y);
        telemetry.addData("r",r);
        telemetry.addData("Theta", Math.toDegrees(Theta));
        telemetry.addData("outputX",outputX);
        telemetry.addData("outputY",outputY);
        telemetry.addData("angle:",adjustAngle(getAbsoluteHeading()));
        telemetry.addData("Angle:",imu.getPosition().z);
        telemetry.update();
        heading = Math.toRadians(getAbsoluteHeading());*/

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

        /* lift
        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
            liftMotor1.setPower(-gamepad2.left_stick_y/2);
            liftMotor2.setPower(-gamepad2.left_stick_y/2);
            liftMotor3.setPower(-gamepad2.left_stick_y/2);
        } else {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
            liftMotor3.setPower(0);

            liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftMotor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }*/

        // Elbow out
        if (gamepad2.a) {
            leftElbow.setPosition(0.15);
            rightElbow.setPosition(0.85);
        }

        // Elbow in
        if (gamepad2.b) {
            leftElbow.setPosition(0.75);
            rightElbow.setPosition(0.25);
        }

        // grabber direction - horizontal
        if (gamepad2.left_bumper) {
            wrist.setPosition(0.25);
        }

        // grabber direction - vertical
        if (gamepad2.right_bumper) {
            wrist.setPosition(0.6);
        }

        // grabber - not grabbing
        if (gamepad2.x) {
            grip.setPosition(0.45);
        }

        // grabber - grabbing
        if (gamepad2.y) {
            grip.setPosition(0.55);
        }

        // lift - up a stage
        if (gamepad2.dpad_up) {
            if (liftstage == 4) { return; } else {
                telemetry.addLine("Going up...");
                telemetry.update();

                pidLiftDrive.setSetpoint(0);
                pidLiftDrive.setOutputRange(0, 1);
                pidLiftDrive.setInputRange(0, 700);
                pidLiftDrive.enable();

                int currentStage = liftstage++;
                int targetPos = (int) (((currentStage - liftstage) * 4) * COUNTS_PER_INCH);

                setLiftMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // setMotorTargetPosition(calcTargetPos(targetPos));
                correction = pidLiftDrive.performPID(liftMotor1.getCurrentPosition());
                setMotorTargetPosition(-targetPos + (int)correction);
                setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(-1);

                while (liftMotor1.getCurrentPosition() != -targetPos) {
                    correction = pidLiftDrive.performPID(liftMotor1.getCurrentPosition());
                    setMotorTargetPosition(-targetPos + (int)correction);
                    setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setLiftMotorPower(-1);
                }

                // while (liftMotor1.isBusy() && liftMotor2.isBusy() && liftMotor3.isBusy()) { }
                // setLiftMotorPower(fightGPower);
                // applyBrakes();
                liftstage = currentStage;
            }
        }

        // lift - down a stage
        if (gamepad2.dpad_down) {
            if (liftstage == 0) { return; } else {
                telemetry.addLine("Going down...");
                telemetry.update();

                pidLiftDrive.setSetpoint(0);
                pidLiftDrive.setOutputRange(0, 1);
                pidLiftDrive.setInputRange(0, 700);
                pidLiftDrive.enable();

                int currentStage = liftstage--;
                int targetPos = (int)(((currentStage - liftstage) * 4) * COUNTS_PER_INCH);

                setLiftMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // setMotorTargetPosition(calcTargetPos(targetPos));
                correction = pidLiftDrive.performPID(liftMotor1.getCurrentPosition());
                setMotorTargetPosition(-targetPos + (int)correction);
                setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                setLiftMotorPower(1);

                while (liftMotor1.getCurrentPosition() != -targetPos) {
                    correction = pidLiftDrive.performPID(liftMotor1.getCurrentPosition());
                    setMotorTargetPosition(-targetPos + (int)correction);
                    setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setLiftMotorPower(-1);
                }

                // while (liftMotor1.isBusy() && liftMotor2.isBusy() && liftMotor3.isBusy()) { }
                // setLiftMotorPower(fightGPower);
                // applyBrakes();
                liftstage = currentStage;
            }
        }

        telemetry.addData("lift1 encoder count", liftMotor1.getCurrentPosition());
        telemetry.addData("lift2 encoder count", liftMotor2.getCurrentPosition());
        telemetry.addData("lift3 encoder count", liftMotor3.getCurrentPosition());

        telemetry.addData("lift1 current", lift1RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lift2 current", lift2RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lift3 current", lift3RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("intake motor", intakeMotorRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.update();

        // intake - counter stall
        try {
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
        }

        // increase lift power by 0.02
        if (gamepad2.dpad_right) {
            if (lPower == 1) { return; } else {
                lPower += 0.02;
                setLiftMotorPower(lPower);
            }
        }

        // decrease lift power by 0.02
        if (gamepad2.dpad_left) {
            if (lPower == 0) { return; } else {
                lPower -= 0.02;
                setLiftMotorPower(lPower);
            }
        }
    }

    private int calcTargetPos(double tPos) {
        while (liftMotor1.getCurrentPosition() != liftMotor1.getTargetPosition()) {
            if (liftMotor1.getCurrentPosition() < liftMotor1.getTargetPosition()) {
                return -((int)tPos - liftMotor1.getCurrentPosition());
            }
        }
        return 0;
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

    private void applyBrakes() {
        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /*private double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    private Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }*/

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