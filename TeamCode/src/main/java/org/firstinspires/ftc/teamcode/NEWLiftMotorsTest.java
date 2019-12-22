/**
 * Only lift test w/ BRAKE MODE
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.openftc.revextensions2.*;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "NEW Motors w/ brake mode", group = "TeleOp")
public class NEWLiftMotorsTest extends OpMode {
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

        // liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");

        // liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // liftEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // liftEx1.setDirection(DcMotorSimple.Direction.REVERSE);
        // liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }

    @Override
    public void loop() {
        /**
         * UPDATE FIRMWARE
         */

        if (gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1) {
            liftMotor1.setPower(gamepad1.left_stick_y);
        } else {
            liftMotor1.setPower(0);
            liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // telemetry.addData("liftTouchSensor", liftTouch.isPressed());
        // telemetry.addData("lift1 encoder count", liftMotor1.getCurrentPosition());
        telemetry.addData("lift1 current", liftRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        // telemetry.addData("Lift stage", liftstage);
        // telemetry.addData("targetPos", targetPos);
        // telemetry.addData("Lift PIDFCoefficients", liftEx1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        // telemetry.addData("lkp", lkp);
        // telemetry.addData("lki", lki);
        // telemetry.addData("lkd", lkd);
        // telemetry.addData("lkf", lkf);
        telemetry.addData("Current Lift Position", liftCurrentPos);

        telemetry.update();
    }
}