package org.firstinspires.ftc.teamcode.Odometry_Monish;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Odometry_Monish.calculations.changeInError;
import static org.firstinspires.ftc.teamcode.Odometry_Monish.calculations.d;
import static org.firstinspires.ftc.teamcode.Odometry_Monish.calculations.dIsNotZero;
import static org.firstinspires.ftc.teamcode.Odometry_Monish.calculations.dIsZero;
import static org.firstinspires.ftc.teamcode.Odometry.OdometryCalculations.coordinatePositionUpdate;

import org.firstinspires.ftc.teamcode.Odometry.OdometryCalculations;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;


@Config
@Autonomous(name = "OdometryLinOpMode", group = "Autonomous")
public class mainDriver extends LinearOpMode {
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;
    private DcMotor intakeMotor1, intakeMotor2, intakeMotor3;
    private Servo foundationServo, foundationServoRight, rightStoneGrabber, grabberLeft, tapeMeasure, liftHoExt, wrist, grabber;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";

    static OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    static OdometryCalculations odometryCalculations;
    Thread updateOdometry;

    final double COUNTS_PER_INCH = 1141.94659527;
    public static double globalHeading, globalXPosEncoderTicks, globalYPosEncoderTicks;

    public static boolean finalPoint = false;
    static double[] powers;
    static double pidOutput;
    public static double verticalLeftPosition, verticalRightPosition, horizontalPosition;

    public static int coordinateNumber = 0;

    public static double lastPoint = 0;

    private double[] xCoordinates1 = {28.6};
    private double [] yCoordinates1 = {4};
    private double [] headings1 = {-90};

    private double[] xCoordinates2 = {23, 20, 28.5};
    private double[] yCoordinates2 = {20, 50, 84};
    private double[] headings2 = {-90, -90, -90};

    private double[] xCoordinates3 = {28.5, 14, 14};
    private double[] yCoordinates3 = {80, 70, 77};
    private double[] headings3 = {-120, -180, -180};

    private double[] xCoordinates4 = {22, 22, 47, 47, 22, 22};
    private double[] yCoordinates4 = {60, 10, -20, -20, 10, 75};
    private double[] headings4 = {-180, -240, -240, -240, -240, -180};

    public void runOpMode() {


        while (!opModeIsActive() && !isStopRequested()) {
            // goToPosition = new MotorPowerMecanum();
            // pid = new PIDCalulations();

            foundationServo = hardwareMap.servo.get("foundationServoLeft");
            foundationServoRight = hardwareMap.servo.get("foundationServoRight");
            rightStoneGrabber = hardwareMap.servo.get("rightStoneGrabber");
            grabberLeft = hardwareMap.servo.get("grabberLeft");
            tapeMeasure = hardwareMap.servo.get("tapeMeasure");
            liftHoExt = hardwareMap.servo.get("liftHoExt");
            wrist = hardwareMap.servo.get("liftGrabberRotater");
            grabber = hardwareMap.servo.get("liftGrabber");

            leftFrontWheel = hardwareMap.dcMotor.get("left front");
            leftBackWheel = hardwareMap.dcMotor.get("left back");
            rightFrontWheel = hardwareMap.dcMotor.get("right front");
            rightBackWheel = hardwareMap.dcMotor.get("right back");
            intakeMotor1 = hardwareMap.dcMotor.get("intake motor 1");
            intakeMotor2 = hardwareMap.dcMotor.get("intake motor 2");
            intakeMotor3 = hardwareMap.dcMotor.get("intake motor 3");
            verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
            verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
            horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

            //Reset the encoders
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeftPosition = verticalLeft.getCurrentPosition();
            verticalRightPosition = verticalRight.getCurrentPosition();
            horizontalPosition = horizontal.getCurrentPosition();

            /*
            Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
            such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
            horizontal encoder travels to the right, it returns positive value
            */
           // verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
           // verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);

            //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Init complete
            telemetry.addData("Status", "Init Complete");
            telemetry.update();

            rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

            coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition);
        }

        if (opModeIsActive()) {
            /*heading = globalPositionUpdate.robotOrientationRadians;
            globalXPosEncoderTicks = globalPositionUpdate.returnXCoordinate();
            globalYPosEncoderTicks = globalPositionUpdate.returnYCoordinate();*/

            foundationDownGrabberUp();
            go(xCoordinates1, yCoordinates1, headings1);
            foundationDownGrabberDown();
            sleep(600);
            foundationUpGrabberDown();
            sleep(300);
            go(xCoordinates2, yCoordinates2, headings2);
            grabFoundation();
            sleep(800);
            go(xCoordinates3, yCoordinates3, headings3);
            releaseFoundation();
            sleep(200);
            intakeOn();
            go(xCoordinates4, yCoordinates4, headings4);

        }
    }

    // set power to each motor
    public void setPower(double lf, double lb, double rf, double rb) {
        leftFrontWheel.setPower(lf);
       leftBackWheel.setPower(lb);
       rightFrontWheel.setPower(rf);
       rightBackWheel.setPower(rb);
    }

    public void go(double[] x, double[] y, double[] heading) {
        do {
            // update global positions
            verticalLeftPosition = verticalLeft.getCurrentPosition();
            verticalRightPosition = verticalRight.getCurrentPosition();
            horizontalPosition = horizontal.getCurrentPosition();


            globalXPosEncoderTicks = coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition)[0];
            globalYPosEncoderTicks = coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition)[1];
            globalHeading = coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition)[2];

            coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition);

            //globalHeading = odometryCalculations.coordinatePositionUpdate()[2];
            //globalXPosEncoderTicks = odometryCalculations.coordinatePositionUpdate()[0];
            //globalYPosEncoderTicks = odometryCalculations.coordinatePositionUpdate()[1];

            // calculate powers and set them to the respective motors
            powers = calculations.goToPositionCalculations(x, y, heading);
            setPower(powers[0]*powers[5], powers[1]*powers[5], powers[2]*powers[5], powers[3]*powers[5]);
            telemetry.addData("globalX", globalXPosEncoderTicks/COUNTS_PER_INCH);
            telemetry.addData("globalY", globalYPosEncoderTicks/COUNTS_PER_INCH);
            telemetry.addData("globalHeading", globalHeading);
            telemetry.addData("when D is 0", dIsZero);
            telemetry.addData("when D is NOT 0", dIsNotZero);
            telemetry.addData("c", powers[4]);
            telemetry.addData("d", d);
            telemetry.addData("changeInError", changeInError);
            telemetry.update();
        } while (powers[6] == 0 || powers[4] > 1 || d > 0.001);

        // stop
        setPower(0, 0, 0, 0);
        coordinateNumber = 0;

    }
    private void foundationDownGrabberUp(){
        foundationServoRight.setPosition(0.92);
        grabberLeft.setPosition(.17);
    }
    private void foundationUpGrabberDown(){
        foundationServoRight.setPosition(.63);  // originally .6
        grabberLeft.setPosition(.65);
    }
    private void foundationDownGrabberDown() {
        foundationServoRight.setPosition(0.92);
        grabberLeft.setPosition(0.65);
    }
    private void grabFoundation() {
        foundationServoRight.setPosition(0.93);
        foundationServo.setPosition(0.28);
        rightStoneGrabber.setPosition(.8);
        grabberLeft.setPosition(.28);
    }
    private void releaseFoundation() {
        foundationServoRight.setPosition(.5);
        foundationServo.setPosition(0.75);
    }
    private void intakeOn() {
        intakeMotor1.setPower(-1);
        intakeMotor2.setPower(1);
        intakeMotor3.setPower(-1);
    }

}