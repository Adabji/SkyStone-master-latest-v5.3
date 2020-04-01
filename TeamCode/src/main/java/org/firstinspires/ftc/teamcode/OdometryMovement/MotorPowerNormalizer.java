package org.firstinspires.ftc.teamcode.OdometryMovement;

import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.trackwidth;
import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.horizontalOffset;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import android.app.WallpaperInfo;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.openftc.revextensions2.*;

import java.util.ArrayList;
import java.util.Locale;
import java.lang.Math;
import java.util.Arrays;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MecanumMotorPowers", group = "Autonomous")
public class MotorPowerNormalizer extends OpMode{
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;


    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //Defining the imu
    BNO055IMU imu;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 1141.94659527;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";

    private static double PosXAngPosY, PosXAngNegY, NegXAng, Theta, c, heading;
    private static double outputX = 0;
    private static double outputY = 0;
    public static double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, powerReduction;
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

        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        heading = globalPositionUpdate.returnOrientation();

        outputX = -Math.cos(heading - Theta) * c;
        outputY = Math.sin(heading - Theta) * c;


        // the negative signs in front of the gamepad inputs may need to be removed.

    }
    /*public static void motorPower (double xPower, double yPower, double turnPower, double reduction){

        c = Math.sqrt(xPower * xPower + yPower * yPower);
        if (yPower < 0) {
            Theta = -Math.atan2(yPower, xPower);
        }

        if (yPower >= 0) {
            Theta = 2 * Math.PI - Math.atan2(yPower, xPower);
        }

        outputX = -Math.cos(heading - Theta) * c;
        outputY = Math.sin(heading - Theta) * c;
        //driveMecanum(xPower, yPower, turnPower, reduction);
    }*/

    public static double[] driveMecanum(double xPower, double yPower, double turnPower, double reduction) {

        c = Math.sqrt(xPower * xPower + yPower * yPower);
        if (yPower < 0) {
            Theta = -Math.atan2(yPower, xPower);
        }

        if (yPower >= 0) {
            Theta = 2 * Math.PI - Math.atan2(yPower, xPower);
        }

        outputX = -Math.cos(heading - Theta) * c;
        outputY = Math.sin(heading - Theta) * c;

        leftFrontPower = outputY + outputX + turnPower;
        leftBackPower = outputY - outputX + turnPower;
        rightFrontPower = outputY - outputX - turnPower;
        rightBackPower = outputY + outputX - turnPower;

        powerReduction = reduction;

        double[] wheelPowers = {Math.abs(rightFrontPower), Math.abs(leftFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];
        if (biggestInput > 1) {
            leftFrontPower /= biggestInput;
            leftBackPower /= biggestInput;
            rightFrontPower /= biggestInput;
            rightBackPower /= biggestInput;


        }

        return wheelPowers;

    }

}
