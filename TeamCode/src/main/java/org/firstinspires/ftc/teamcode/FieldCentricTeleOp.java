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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.*;

import java.util.Locale;
import java.lang.Math;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric Mecanum", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    private static double JoyStickAngleRad;
    private static double JoyStickAngleDeg;
    private static DcMotor leftFrontWheel;
    private static DcMotor leftBackWheel;
    private static DcMotor rightFrontWheel;
    private static DcMotor rightBackWheel;
    private static DcMotor intakeMotor;
    private static DcMotor liftMotor1;
    private static DcMotor liftMotor2;
    private static DcMotor liftMotor3;
    private static ExpansionHubMotor intakeMotorRE2;
    private static ExpansionHubMotor lift1RE2;
    private static ExpansionHubMotor lift2RE2;
    private static ExpansionHubMotor lift3RE2;
    private static double PosXAngPosY;
    private static double PosXAngNegY;
    private static double NegXAng;
    private static double Theta;
    private static double r;
    private static double outputX;
    private static double outputY;
    private static double heading;
    Double Deg = Math.PI;
    BNO055IMU imu;
    Orientation angles;
    double zeroPos = 0;


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


        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
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
        telemetry.addData("LeftX",gamepad1.left_stick_x);
        telemetry.addData("LeftY", -gamepad1.left_stick_y);
        telemetry.addData("r",r);
        telemetry.addData("Theta", Math.toDegrees(Theta));
        telemetry.addData("outputX",outputX);
        telemetry.addData("outputY",outputY);
        telemetry.addData("angle:",adjustAngle(getAbsoluteHeading()));
        //telemetry.addData("Angle:",imu.getPosition().z);
        telemetry.update();
        heading = Math.toRadians(getAbsoluteHeading());
        if (gamepad2.a){
            heading = 0;
        }

        if (gamepad1.a) {
            intakeMotor.setPower(1);
        }

        if (gamepad1.b || gamepad1.x) {
            intakeMotor.setPower(0);
        }

        if (gamepad1.y) {
            intakeMotor.setPower(-1);
        }

        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
            liftMotor1.setPower(-gamepad2.left_stick_y);
            liftMotor2.setPower(-gamepad2.left_stick_y);
            liftMotor3.setPower(-gamepad2.left_stick_y);
        } else {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
            liftMotor3.setPower(0);

            liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftMotor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("lift1 current", lift1RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lift2 current", lift2RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lift3 current", lift3RE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));

        try {
            if (intakeMotorRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 5) {
                telemetry.addLine("Intake Motor stalling. Restarting motor...");
                telemetry.update();
                intakeMotor.setPower(0);
                Thread.sleep(500);
                intakeMotor.setPower(-1);
                Thread.sleep(500);
                intakeMotor.setPower(1);
            }
        } catch (InterruptedException ie) {
            telemetry.addLine("Thread interrupted. Exiting...");
            telemetry.update();
        }
    }
    public double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    public double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }
    Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public static void driveMecanum(double forwards, double horizontal, double turning) {
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