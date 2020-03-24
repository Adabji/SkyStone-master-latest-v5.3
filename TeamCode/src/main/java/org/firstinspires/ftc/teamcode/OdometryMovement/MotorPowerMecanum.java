package org.firstinspires.ftc.teamcode.OdometryMovement;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerNormalizer;
import java.util.Locale;
import java.lang.Math;
import java.util.Arrays;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerNormalizer.motorPower;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MecanumMotorPowers", group = "Autonomous")
    public class MotorPowerMecanum extends OpMode {

    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    public double heading, globalXPosEncoderTicks, globalYPosEncoderTicks, desiredXCoordinate, desiredYCoordinate, desiredHeading;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //Defining the imu
    BNO055IMU imu;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 1141.94659527;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";


    public void init(){
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


}
    @Override
    public void loop() {

        //Importing odometry readings
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        heading = globalPositionUpdate.returnOrientation();
        globalXPosEncoderTicks = globalPositionUpdate.returnXCoordinate();
        globalYPosEncoderTicks = globalPositionUpdate.returnYCoordinate();
        goToPositionCalculations(4,5,6);

        // the negative signs in front of the gamepad inputs may need to be removed.

    }



    public void goToPositionCalculations (double desiredXCoordinate, double desiredYCoordinate, double desiredHeading){

        //Converting the odometry readings in encoder ticks to inches
        double globalXPos = globalXPosEncoderTicks/COUNTS_PER_INCH;
        double globalYPos = globalYPosEncoderTicks/COUNTS_PER_INCH;

        //Getting the ratio of motor powers based off the distance to target in each axis
        double xPowerRatio = desiredXCoordinate - globalXPos;
        double yPowerRatio = desiredYCoordinate - globalYPos;

        //Finding the reduction factor based off the distance to target
        double distanceTotarget = Math.sqrt(xPowerRatio*xPowerRatio+yPowerRatio*yPowerRatio);
        double proportionPowerReduction = distanceTotarget/25;

        //Setting the turning power temporarily to 0
        double turnPower = 0;

        if (proportionPowerReduction > 1) {
            proportionPowerReduction = 1;
        }

        motorPower(xPowerRatio, yPowerRatio, turnPower, proportionPowerReduction);


    }

   /* public double returngoToPositionCalculations(){ return goToPositionCalculations(desiredXCoordinate, desiredYCoordinate, desiredHeading); }*/

}
