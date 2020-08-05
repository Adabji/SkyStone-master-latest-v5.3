package org.firstinspires.ftc.teamcode.OdometryMovement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.c;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.leftBackPower;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.leftFrontPower;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.pointCounter;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.reachedPointSignal;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.rightBackPower;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.rightFrontPower;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.rotationalDistance;
import static org.firstinspires.ftc.teamcode.OdometryMovement.PIDCalulations.pidOutput;

public class GoToPositionLinearOp extends LinearOpMode {
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    MotorPowerMecanum goToPosition;
    PIDCalulations pid;

    public double currentTime, previousTime;
    public double currentError, previousError;
    /*public double p;
    public double d;
    public double pidOutput;*/
    public double xError;
    public double yError;
    public double[] desiredXCoordinates = {70, 60, 0};
    public double[] desiredYCoordinates = {25, 15, 15};
    public double[] desiredHeading = {180, 180, 90};


    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    final double COUNTS_PER_INCH = 1141.94659527;
    public static double heading, globalXPosEncoderTicks, globalYPosEncoderTicks;

    public void init() {

        goToPosition = new MotorPowerMecanum();
        pid = new PIDCalulations();

        pointCounter = 0;
        reachedPointSignal = 0;
        c = 100;

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
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

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


        /*while (c < 1) {

        goToPosition.goToPositionCalculations(25, 25, 180);
    }
        c = 4;

        while (c < 1) {

        goToPosition.goToPositionCalculations(25, 40, 200);
    }*/


    @Override
    while(opModeIsActive) {

        //MotorPowerMecanum goToPosition = new MotorPowerMecanum();
        //goToPosition.goToPositionCalculations(-15, -15, 0);

        heading = globalPositionUpdate.robotOrientationRadians;
        globalXPosEncoderTicks = globalPositionUpdate.returnXCoordinate();
        globalYPosEncoderTicks = globalPositionUpdate.returnYCoordinate();


        goToPosition.goToPositionCalculations(desiredXCoordinates, desiredYCoordinates, desiredHeading);
        pid.pidCalculations(c);


        leftFrontWheel.setPower(leftFrontPower * pidOutput);
        rightFrontWheel.setPower(rightFrontPower * pidOutput);
        leftBackWheel.setPower(leftBackPower * pidOutput);
        rightBackWheel.setPower(rightBackPower * pidOutput);


        telemetry.addData("c", c);
        telemetry.addData("rotationalDistance", rotationalDistance);
        telemetry.update();

    }
}
