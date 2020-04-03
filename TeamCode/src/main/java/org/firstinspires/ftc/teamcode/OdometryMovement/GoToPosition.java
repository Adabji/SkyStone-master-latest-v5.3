package org.firstinspires.ftc.teamcode.OdometryMovement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerNormalizer.*;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerNormalizer.*;
import org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

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
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerNormalizer.driveMecanum;
import org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.*;




@Config
@Autonomous(name = "OdometryGoToPosition", group = "Autonomous")

    public class GoToPosition extends OpMode {

        private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

        //Odometry encoder wheels
        DcMotor verticalRight, verticalLeft, horizontal;

    MotorPowerMecanum goToPosition;


        //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
        String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";


        public void init() {

            goToPosition = new MotorPowerMecanum();

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

            rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        }




        @Override
        public void loop() {

            //MotorPowerMecanum goToPosition = new MotorPowerMecanum();
            //goToPosition.goToPositionCalculations(-15, -15, 0);

            goToPosition.goToPositionCalculations(-15, -15, 0);

            leftFrontWheel.setPower(leftFrontPower*powerReduction);
            rightFrontWheel.setPower(rightFrontPower*powerReduction);
            leftBackWheel.setPower(leftBackPower*powerReduction);
            rightBackWheel.setPower(rightBackPower*powerReduction);

                telemetry.addData("leftFrontPower", leftFrontPower*powerReduction);
                telemetry.addData("rightFrontPower", rightFrontPower*powerReduction);
                telemetry.addData("leftBackPower", leftBackPower*powerReduction);
                telemetry.addData("rightBackPower", rightBackPower*powerReduction);
                telemetry.update();


        }


    }

