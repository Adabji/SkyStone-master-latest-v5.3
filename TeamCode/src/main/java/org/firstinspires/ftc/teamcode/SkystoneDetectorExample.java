package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Locale;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */
@Autonomous(name = "Skystone Detector OpMode", group="Autonomous")

public class SkystoneDetectorExample extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    // hardware stuff
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, intakeMotor, liftMotor1;
    private static ExpansionHubMotor intakeMotorRE2, liftRE2;
    private static DcMotorEx liftEx1;
    private static Servo leftElbow, rightElbow, wrist, grip, foundationServoLeft, foundationServoRight;
    private TouchSensor liftTouch;
    private TouchSensor intakeTouch;
    BNO055IMU imu;
    PIDController           pidRotate, pidDrive;
    double                  globalAngle, power = .30, correction, rotation;
    double                  rotationPower = 0.5;
    double                  movePower = 0.7;
    Orientation             lastAngles = new Orientation();
    private DistanceSensor colorSensorDistance;

    // PIDF stuff
    PIDFCoefficients pidfCoefficients;
    double lkp = 0;
    double lki = 0;
    double lkd = 0;
    double lkf = 0;

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

    // Camera stuff
    String skystoneLoc = "";

    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        // hardware stuff
        /*leftFrontWheel = hardwareMap.dcMotor.get("left front");
        leftBackWheel = hardwareMap.dcMotor.get("left back");
        rightFrontWheel = hardwareMap.dcMotor.get("right front");
        rightBackWheel = hardwareMap.dcMotor.get("right back");

        intakeMotor = hardwareMap.dcMotor.get("intake motor");
        intakeMotorRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake motor");

        liftMotor1 = hardwareMap.dcMotor.get("lift1");
        liftRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift1");
        liftEx1 = (DcMotorEx) hardwareMap.dcMotor.get("lift1");

        colorSensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

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
        // liftEx1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        foundationServoLeft.setPosition(0.2);
        foundationServoRight.setPosition(0.87);
        // leftElbow.setPosition(0.85);
        // rightElbow.setPosition(0.15);
        wrist.setPosition(0.6);
        grip.setPosition(0.49);*/

        /*
         * Wait for the user to press start on the Driver Station
         */
        while (!opModeIsActive() && !isStopRequested()) {
            if (skyStoneDetector.isDetected()) {
                if (skyStoneDetector.getScreenPosition().x < 100) {
                    skystoneLoc = "left";
                } else if (skyStoneDetector.getScreenPosition().x < 190 && skyStoneDetector.getScreenPosition().x > 100) {
                    skystoneLoc = "center";
                } else {
                    skystoneLoc = "right";
                }
            }

            telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition().x, skystoneLoc);
            telemetry.addData("Status", "Waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Skystone Location = " + skyStoneDetector.getScreenPosition().x, skystoneLoc);
            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            // telemetry.addData("Frame Count", phoneCam.getFrameCount());
            // telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            // telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            // telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            // telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            // telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            // FIRST SKYSTONE - OUTER ONE
            move(1, movePower/2, true);
            if (skystoneLoc.equals("right")) {
                strafe(11, movePower/1.5, true);
            } else if (skystoneLoc.equals("center")) {
                strafe(3, movePower/1.5, true);
            }

            rotate(90, rotationPower);
            strafe(25, movePower, true);
            move(1, movePower/2, true);

            do {
                intakeMotor.setPower(-1);
            } while (!intakeTouch.isPressed());

            intakeMotor.setPower(0);
            // elbow out

            // move until foundation in sight
            // WHAT IF FOUNDATION IS NOT MOVED
            // FOCUS ON JUST MOVING THEM FIRST
            /*
             * if (inchesc > 44 or something) {
             *  turnLeft 90 deg
             *  foundation Servo Up
             *  moveForward a bit
             *  foundation Servo Down
             *  pull back in a curve
             *  push forward
             *  release grabber
             *  go for second stone or stop on the line
             * }
             */
            boolean inSight = false;
            while (!inSight) {
                setMotorPower(-movePower, -movePower, -movePower, -movePower);
                if (colorSensorDistance.getDistance(DistanceUnit.INCH) < 6) {
                    inSight = true;
                }
            }

            while (leftFrontWheel.isBusy() && leftBackWheel.isBusy() && rightFrontWheel.isBusy() && rightBackWheel.isBusy()) { }
            setMotorPower(0, 0, 0, 0);

            // deposit stone
        }
    }

    private void setMotorMode(DcMotor.RunMode m) {
        leftFrontWheel.setMode(m);
        leftBackWheel.setMode(m);
        rightFrontWheel.setMode(m);
        rightBackWheel.setMode(m);
    }

    private void setMotorPower(double lf, double lb, double rf, double rb) {
        leftFrontWheel.setPower(lf);
        leftBackWheel.setPower(lb);
        rightFrontWheel.setPower(rf);
        rightBackWheel.setPower(rb);
    }

    private void applyBrakes() {
        leftFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void setMotorTargetPosition(int lf, int lb, int rf, int rb) {
        leftFrontWheel.setTargetPosition(lf);
        leftBackWheel.setTargetPosition(lb);
        rightFrontWheel.setTargetPosition(rf);
        rightBackWheel.setTargetPosition(rb);
    }

    // set direction to true if strafing right, false if strafing left
    private void strafe(double distance, double power, boolean direction) {
        if(distance == 0) return;

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
        setMotorPower(myPower, -myPower, -myPower, myPower);

        while (leftFrontWheel.isBusy() && leftBackWheel.isBusy() && rightFrontWheel.isBusy() && rightBackWheel.isBusy()) { }
        setMotorPower(0, 0, 0, 0);
        applyBrakes();
    }

    private void move(double distance, double power, boolean direction) {
        if(distance == 0) return;

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
        setMotorTargetPosition(moveDistance, moveDistance, moveDistance, moveDistance);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(myPower, myPower, myPower, myPower);

        while (leftFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy() && rightFrontWheel.isBusy()) { }
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
                setMotorPower(power, power, -power, -power);
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
        setMotorPower(0, 0,0, 0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}