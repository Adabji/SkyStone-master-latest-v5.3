package org.firstinspires.ftc.teamcode.Odometry_Monish;

import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Odometry_Monish.mainDriver.globalXPosEncoderTicks;
import static org.firstinspires.ftc.teamcode.Odometry_Monish.mainDriver.globalYPosEncoderTicks;
import static org.firstinspires.ftc.teamcode.Odometry_Monish.mainDriver.globalHeading;

public class calculations {
    public static double Theta, c, linearDistance, rotationalDistance;
    public static double outputX;
    public static double outputY, headingForTurning;
    public static double reductionDistance;
    public static double distanceToTurn;
    public static double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, powerReduction;
    public static double circumference = 64.42;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final static double COUNTS_PER_INCH = 1141.94659527;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    public static double globalXPos, globalYPos, proportionPowerReduction, turnPower;
    public static double xPowerRatio, yPowerRatio;

    public static double previousError, previousTime, currentTime, p, i, d, pidOutput;

    public static double pidCalculations(double currentError){

        currentTime = System.currentTimeMillis();

        p = currentError / 18;
        d = ((currentError - previousError) / (currentTime - previousTime)) / 100;
        i = 0;

        pidOutput = Range.clip((p + i + d), 0, 1);

        previousError = currentError;
        previousTime = currentTime;

        return pidOutput;
    }

    public static double[] goToPositionCalculations (double desiredXCoordinate, double desiredYCoordinate, double desiredHeading) {
        //Converting the odometry readings in encoder ticks to inches
        globalXPos = globalXPosEncoderTicks / COUNTS_PER_INCH;
        globalYPos = globalYPosEncoderTicks / COUNTS_PER_INCH;

        //Getting the ratio of motor powers based off the distance to target in each axis
        xPowerRatio = (desiredXCoordinate - globalXPos);
        yPowerRatio = (desiredYCoordinate - globalYPos);

        //Finding the reduction factor based off the distance to target
        reductionDistance = Range.clip(c, 0, 25);
        proportionPowerReduction = Range.clip(Math.sqrt(Math.abs(c / 25)), 0, 1);

        if (globalHeading >= 0) {
            headingForTurning = globalHeading;
        }
        if (globalHeading < 0) {
            headingForTurning = globalHeading + 360;
        }

        distanceToTurn = desiredHeading - Math.toDegrees(globalHeading);
        turnPower = distanceToTurn / 360 * circumference;
        return driveMecanum(xPowerRatio, yPowerRatio, turnPower, 0);
    }

    public static double[] driveMecanum(double xPower, double yPower, double turnPower, double reduction) {
        rotationalDistance = Math.abs((distanceToTurn/360)*circumference);
        linearDistance = Math.sqrt(xPower * xPower + yPower * yPower);
        c = linearDistance+ Math.abs((distanceToTurn/360)*circumference);

        Theta = Math.atan2(xPower, yPower);

        outputY = Math.cos(globalHeading - Theta)*linearDistance;
        outputX = -Math.sin(globalHeading - Theta)*linearDistance;

        leftFrontPower = (outputY + outputX) + turnPower;
        leftBackPower = (outputY - outputX) + turnPower;
        rightFrontPower = (outputY - outputX) - turnPower;
        rightBackPower = (outputY + outputX) - turnPower;

        powerReduction = reduction;

        double[] wheelPowers = {Math.abs(rightFrontPower), Math.abs(leftFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];

        leftFrontPower /= biggestInput;
        leftBackPower /= biggestInput;
        rightFrontPower /= biggestInput;
        rightBackPower /= biggestInput;

        return new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower, c};
    }
}
