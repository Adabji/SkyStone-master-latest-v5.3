package org.firstinspires.ftc.teamcode.Odometry_Monish;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import org.firstinspires.ftc.teamcode.Odometry_Monish.calculations;

@Config
@Autonomous(name = "OdometryLinOpMode", group = "Autonomous")
public class mainDriver extends LinearOpMode {
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    final double COUNTS_PER_INCH = 1141.94659527;
    public static double heading, globalXPosEncoderTicks, globalYPosEncoderTicks;

    public void runOpMode() {
        while (!opModeIsActive() && !isStopRequested()) {
            // goToPosition = new MotorPowerMecanum();
            // pid = new PIDCalulations();

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

        if (opModeIsActive()) {
            heading = globalPositionUpdate.robotOrientationRadians;
            globalXPosEncoderTicks = globalPositionUpdate.returnXCoordinate();
            globalYPosEncoderTicks = globalPositionUpdate.returnYCoordinate();

            double[] powers = calculations.goToPositionCalculations(25, 25, 180);

            setPower(powers[0], powers[1], powers[2], powers[3], powers[4]);
        }
    }

    /*telemetry.addData("heading", heading);
    telemetry.addData("Theta",Theta);
    telemetry.addData("p", p);
    telemetry.addData("d", d);
    telemetry.addData("headingForTurning", headingForTurning);
    telemetry.addData("distanceToTurn", distanceToTurn);
    telemetry.addData("leftFrontPower", leftFrontPower*pidOutput);
    telemetry.addData("rightFrontPower", rightFrontPower*pidOutput);
    telemetry.addData("leftBackPower", leftBackPower*pidOutput);
    telemetry.addData("rightBackPower", rightBackPower*pidOutput);
    telemetry.addData("xPowerRatio", xPowerRatio);
    telemetry.addData("yPowerRatio", yPowerRatio);
    telemetry.addData("c", c);
    telemetry.update();*/

    public static void setPower(double lf, double lb, double rf, double rb, double c) {
        double pidOutput = calculations.pidCalculations(c);
        leftFrontWheel.setPower(lf*pidOutput);
        rightFrontWheel.setPower(rf*pidOutput);
        leftBackWheel.setPower(lb*pidOutput);
        rightBackWheel.setPower(rb*pidOutput);
    }
}