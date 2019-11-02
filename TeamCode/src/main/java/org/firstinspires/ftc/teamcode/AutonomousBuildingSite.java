/**
 * Autonomous on the blue building site side.
 *
 * NOTES:
 * - Fix the irregular movement part
 * - Bump into the wall to fix the angle (try again if PID loops counter this)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.openftc.revextensions2.*;

@Autonomous(name="Autonomous Building Site", group="Autonomous")
public class AutonomousBuildingSite extends LinearOpMode {
    DcMotor                 leftFront, leftBack, rightFront, rightBack, intakeMotor;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    PIDController           pidRotate, pidDrive;
    double                  globalAngle, power = .30, correction, rotation;
    double                  rotationPower = 0.5;
    double                  movePower = 0.4;    // this is a slow move speed--required for this auto to prevent the robot from smashing into things
    static final double     COUNTS_PER_MOTOR_REV  = 537.6;
    static final double     DRIVE_GEAR_REDUCTION  = 1.0;
    static final double     WHEEL_DIAMETER_INCHES = 3.937;
    static final double     COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    ExpansionHubMotor intakeMotorRE2;
    DistanceSensor leftRange, rightRange;
    private ColorSensor sensorColor;
    private DistanceSensor sensorColorDistance;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        intakeMotor = hardwareMap.dcMotor.get("intake motor");

        intakeMotorRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake motor");

        /*leftRange = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightRange= hardwareMap.get(DistanceSensor.class, "right_distance");*/

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        applyBrakes();

        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        applyBrakes();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Waiting for start command...");
            telemetry.update();
        }

        sleep(1000);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        if (opModeIsActive()) {

            // Use gyro to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            // Moving to the foundation, pulling it, and then moving to the line

            move(5, movePower/2, false);
            // foundationServo.setPosition(0.5);
            strafe(24, movePower, true);

            // move(25, movePower, false);

            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            boolean inSight = false;
            while (!inSight) {
                setMotorPower(-movePower, -movePower, -movePower, -movePower);

                if (sensorColorDistance.getDistance(DistanceUnit.INCH) < 4) {
                    inSight = true;
                }
            }

            while(leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) { }

            setMotorPower(0, 0, 0, 0);
            applyBrakes();

            Thread.sleep(300);
            move(3, movePower/3, false);
            // foundationServo.setPosition(0);
            Thread.sleep(300);
            move(27, movePower/1.7, true);
            // foundationServo.setPosition(0.5);
            Thread.sleep(15000);
            strafe(20, movePower, false);
            move(5, movePower, false);
            // foundationServo.setPosition(0);
            strafe(28, movePower, false);
        }
    }

    private void setMotorMode(DcMotor.RunMode m) {
        leftFront.setMode(m);
        leftBack.setMode(m);
        rightFront.setMode(m);
        rightBack.setMode(m);
    }

    private void setMotorPower(double lf, double lb, double rf, double rb) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    private void applyBrakes() {
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void setMotorTargetPosition(int lf, int lb, int rf, int rb) {
        leftFront.setTargetPosition(lf);
        leftBack.setTargetPosition(lb);
        rightFront.setTargetPosition(rf);
        rightBack.setTargetPosition(rb);
    }

    // set direction to true if strafing right, false if strafing left
    private void strafe(int distance, double power, boolean direction) {
        if(distance == 0) return;

        telemetry.addData("strafing direction", direction);
        telemetry.update();

        int targetPos = (int)(distance * COUNTS_PER_INCH);

        double myPower;
        int strafeDistance;

        if(direction) {
            myPower = power;
            strafeDistance = targetPos;
        } else {
            myPower = -power;
            strafeDistance = -targetPos;
        }

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorTargetPosition(strafeDistance, -strafeDistance, -strafeDistance, strafeDistance);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(myPower + correction, -myPower - correction, -myPower - correction, myPower + correction);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) { }

        setMotorPower(0, 0, 0, 0);

        applyBrakes();
    }

    private void move(int distance, double power, boolean direction) {
        if(distance == 0) return;

        telemetry.addData("moving direction", direction);
        telemetry.update();

        int targetPos = (int)(distance * COUNTS_PER_INCH);

        double myPower;
        int moveDistance;

        if(direction) {
            myPower = power;
            moveDistance = targetPos;
        } else {
            myPower = -power;
            moveDistance = -targetPos;
        }

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorTargetPosition(moveDistance, moveDistance,moveDistance, moveDistance);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(myPower + correction, myPower + correction, myPower + correction, myPower + correction);

        while(leftFront.isBusy() && leftBack.isBusy() && rightBack.isBusy() && rightFront.isBusy()) { }

        setMotorPower(0, 0, 0, 0);

        applyBrakes();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction *= gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                setMotorPower(power, power, -power, power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                setMotorPower(-power, -power, power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                setMotorPower(-power, -power, power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        setMotorPower(0, 0, 0, 0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}