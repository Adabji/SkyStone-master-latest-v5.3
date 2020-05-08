package org.firstinspires.ftc.teamcode.OdometryMovement;

import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.trackwidth;
import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.horizontalOffset;
import static org.firstinspires.ftc.teamcode.OdometryMovement.MotorPowerMecanum.c;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import android.app.WallpaperInfo;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PIDCalulations {

    public static double previousError, previousTime, currentTime, p, i, d, pidOutput;

    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    DcMotor verticalRight, verticalLeft, horizontal;

    public void init() {

    }
    public void loop() {

    }

    public void pidCalculations(double currentError){

        //while (c > 0.1) {

            currentTime = System.currentTimeMillis();

            p = currentError / 18;
            d = ((currentError - previousError) / (currentTime - previousTime)) / 100;
            i = 0;

            pidOutput = Range.clip((p + i + d), 0, 1);

            previousError = currentError;
            previousTime = currentTime;


        //}
    }
}
