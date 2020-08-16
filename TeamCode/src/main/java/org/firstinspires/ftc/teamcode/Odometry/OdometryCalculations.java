package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import static android.os.Looper.loop;
import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.horizontalOffset;
import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.trackwidth;

public class OdometryCalculations implements Runnable{

    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;
    private double verticalLeftEncoderWheelPosition, verticalRightEncoderWheelPosition, previousVerticalLeftEncoderWheelPosition, previousVerticalRightEncoderWheelPosition;
    private double changeInRobotOrientation, headingRadians, normalEncoderWheelPosition,prevNormalEncoderWheelPosition;
    public double COUNTS_PER_INCH = 1141.94659527;
    public double robotEncoderWheelDistance = trackwidth*COUNTS_PER_INCH;
    public double globalXPos, globalYPos;
    public boolean isRunning = true;




    public double[] coordinatePositionUpdate(){
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition()*-1);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition());

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        headingRadians = ((headingRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition());
        //double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        //double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double rawHorizontalChange = (normalEncoderWheelPosition - prevNormalEncoderWheelPosition);
        double horizontalChange = rawHorizontalChange - changeInRobotOrientation*horizontalOffset*1141.94659527;

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        globalXPos = globalXPos + (p*Math.sin(headingRadians) + n*Math.cos(headingRadians));
        globalYPos = globalYPos - (p*Math.cos(headingRadians) - n*Math.sin(headingRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;



        return new  double[] {globalXPos, globalYPos, -headingRadians};
    }
    public void run(){
        while(isRunning){
            coordinatePositionUpdate();
        }
    }
}
