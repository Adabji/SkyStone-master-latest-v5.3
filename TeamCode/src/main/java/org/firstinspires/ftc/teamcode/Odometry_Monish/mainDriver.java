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

import org.firstinspires.ftc.teamcode.Odometry.OdometryCalculations;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;


@Config
@Autonomous(name = "OdometryLinOpMode", group = "Autonomous")
public class mainDriver extends LinearOpMode {
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;
    private Servo foundationServo, foundationServoRight, rightStoneGrabber, grabberLeft, tapeMeasure, liftHoExt, wrist, grabber;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";

    static OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    static OdometryCalculations odometryCalculations;
    Thread updateOdometry;

    final double COUNTS_PER_INCH = 1141.94659527;
    public static double globalHeading, globalXPosEncoderTicks, globalYPosEncoderTicks;

    static double[] powers;
    static double pidOutput;

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
            verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
            verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
            horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

            //Reset the encoders
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
            positionThread = new Thread(globalPositionUpdate);
            positionThread.start();


            globalPositionUpdate.reverseLeftEncoder();

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
        }

        if (opModeIsActive()) {
            /*heading = globalPositionUpdate.robotOrientationRadians;
            globalXPosEncoderTicks = globalPositionUpdate.returnXCoordinate();
            globalYPosEncoderTicks = globalPositionUpdate.returnYCoordinate();*/

            //foundationDownGrabberUp();
            go(90, 24, 180);
            //foundationDownGrabberDown();

        }
    }

    // set power to each motor
    public void setPower(double lf, double lb, double rf, double rb) {
        leftFrontWheel.setPower(lf);
        leftBackWheel.setPower(lb);
        rightFrontWheel.setPower(rf);
        rightBackWheel.setPower(rb);
    }

    public void go(double x, double y, double heading) {
        do {
            // update global positions
            globalHeading = globalPositionUpdate.returnOrientation() /*+(Actual Starting Position)*/;
            globalXPosEncoderTicks = globalPositionUpdate.returnXCoordinate();
            globalYPosEncoderTicks = globalPositionUpdate.returnYCoordinate();

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
        } while (powers[4] > -1/*powers[4] > 1.5 || d > 0.001*/);

        // stop
        while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()) {}
        setPower(0, 0, 0, 0);

    }
    private void foundationDownGrabberUp(){
        foundationServoRight.setPosition(0.96);
        grabberLeft.setPosition(.2);
    }
    private void foundationUpGrabberDown(){
        foundationServoRight.setPosition(.63);  // originally .6
        grabberLeft.setPosition(.7);
    }
    private void foundationDownGrabberDown() {
        foundationServoRight.setPosition(1);
        grabberLeft.setPosition(.78);
    }
}