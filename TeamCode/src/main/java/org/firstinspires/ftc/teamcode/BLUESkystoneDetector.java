package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.CbColorFilter;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class BLUESkystoneDetector extends SkystoneDetector {
    // Results of the detector
    private Point screenPosition = new Point(); // Screen position of the mineral
    private Rect foundRect = new Rect(); // Found rect

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();

    private Rect fullWidthRect = new Rect();

    private double foundBestDifference = Double.MAX_VALUE;

    // Skystone

    public static Telemetry skystoneTel = null;



    public Point getScreenPosition() {
        return screenPosition;
    }

    @Override
    public Mat process(Mat input) {
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);


        yellowFilter.process(workingMat.clone(), yellowMask);

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        /*blackFilter.process(workingMat.clone(), blackMask);
        blackFilter.process(input.clone(), blackMask);
        blackMask.copyTo(displayMat);*/


        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursYellow, -1, new Scalar(255,30,30), 2);


        // Current result
        Rect bestRect = foundRect.clone();
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better



        // Loop through the contours and score them, searching for the best result
        /*for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            skystoneTel.addData("score", score);
            skystoneTel.addData("rect", rect);
            skystoneTel.update();

            try {
                Thread.sleep(2000);
            } catch (InterruptedException i) {

            }

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDifference){
                bestDifference = score;
                bestRect = rect;
            }
        }


        Imgproc.rectangle(blackMask, bestRect.tl(), bestRect.br(), new Scalar(255,0,255), 1, Imgproc.LINE_4, 0);*/
        blackFilter.process(workingMat.clone(), blackMask);


        // new
        // blackMask.copyTo(displayMat);

        List<MatOfPoint> contoursBlack = new ArrayList<>();

        Imgproc.findContours(blackMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursBlack, -1, new Scalar(40,40,40), 2);

        for(MatOfPoint cont : contoursBlack) {
            cont.adjustROI(0, 0, 70, 0);
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255), 2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
            }
        }

        /*skystoneTel.addData("foundbestDiff", foundBestDifference);
        skystoneTel.addData("currentBestDiff", bestDifference);
        skystoneTel.addData("fullWidthRectX", fullWidthRect.x);
        skystoneTel.addData("fullWidthRectY", fullWidthRect.y);
        skystoneTel.addData("fullWidthRectWidth", fullWidthRect.width);

        skystoneTel.addData("currentBestRect", bestRect);
        skystoneTel.addData("fullWidthRectY", fullWidthRect.y);
        skystoneTel.addData("fullWidthRectWidth", fullWidthRect.width);
        skystoneTel.update();

        try {
            Thread.sleep(2000);
        } catch (InterruptedException i) {

        }*/

        if(bestRect != null && foundBestDifference > bestDifference) {
            foundRect = bestRect.clone();
            foundBestDifference = bestDifference;
            found = true;
        }

        if (foundRect != null) {
            fullWidthRect.x = 0;
            fullWidthRect.width = input.width();
            fullWidthRect.y = foundRect.y;
            fullWidthRect.height = foundRect.height;

            // Show chosen result
            Imgproc.rectangle(displayMat, foundRect.tl(), foundRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", foundRect.tl(),0,1,new Scalar(255,255,255));
            // Imgproc.rectangle(displayMat, fullWidthRect.tl(), fullWidthRect.br(), new Scalar(0,255,0), 4);

            screenPosition = foundRect.tl();
        } else {
            found = false;
        }

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }

    public double calculateScore(Mat input) {
        if(!(input instanceof MatOfPoint)) return Double.MAX_VALUE;
        MatOfPoint contour = (MatOfPoint) input;
        double score = Double.MAX_VALUE;

        // Get bounding rect of contour
        Rect rect = Imgproc.boundingRect(contour);
        if (rect.y > 160 && rect.width > 10) { score = Math.abs(rect.height - 44); }

        return score;
    }

    public void setFoundToFalse() {
        found = false;
        foundBestDifference = Double.MAX_VALUE;
    }
}
