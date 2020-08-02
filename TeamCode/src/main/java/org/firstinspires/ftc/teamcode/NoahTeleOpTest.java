package org.firstinspires.ftc.teamcode;

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
import org.openftc.revextensions2.*;

import java.util.Locale;
import java.lang.Math;
import java.util.Arrays;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Noah Tele-Op  Test", group = "TeleOp")

public class NoahTeleOpTest extends OpMode{

    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    //Creating variables for the joystick positions
    private static double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;

    //Variables for driving
    private static double angleOfTravel, rightFrontPower, rightBackPower, leftFrontPower, leftBackPower;

    @Override
    public void init(){

        //initializing motors
        leftFrontWheel = hardwareMap.dcMotor.get("left front");
        leftBackWheel = hardwareMap.dcMotor.get("left back");
        rightFrontWheel = hardwareMap.dcMotor.get("right front");
        rightBackWheel = hardwareMap.dcMotor.get("right back");

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop () {

        //setting the variables to the joystick positions
        leftJoystickX = gamepad1.left_stick_x;
        leftJoystickY = gamepad1.left_stick_y;
        rightJoystickX = gamepad1.right_stick_x;
        rightJoystickY = gamepad1.right_stick_y;

        angleOfTravel = Math.toDegrees(Math.atan2(leftJoystickX, leftJoystickY));

        leftFrontPower = (leftJoystickY - leftJoystickX) - rightJoystickX;
        rightBackPower = (leftJoystickY - leftJoystickX) + rightJoystickX;
        rightFrontPower = (leftJoystickY + leftJoystickX) + rightJoystickX;
        leftBackPower = (leftJoystickY + leftJoystickX) - rightJoystickX;

        double[] wheelPowers = {Math.abs(leftFrontPower), Math.abs(rightBackPower), Math.abs(rightFrontPower), Math.abs(leftBackPower)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];

         if (biggestInput > 1) {

             leftFrontPower /= biggestInput;
             leftBackPower /= biggestInput;
             rightFrontPower /= biggestInput;
             rightBackPower /= biggestInput;

         }

         if (gamepad1.right_trigger > 0.5) {

             leftFrontWheel.setPower(leftFrontPower/2);
             rightFrontWheel.setPower(rightFrontPower/2);
             leftBackWheel.setPower(leftBackPower/2);
             rightBackWheel.setPower(rightBackPower/2);

         } else {

             leftFrontWheel.setPower(leftFrontPower);
             rightFrontWheel.setPower(rightFrontPower);
             leftBackWheel.setPower(leftBackPower);
             rightBackWheel.setPower(rightBackPower);

         }

    }

}
