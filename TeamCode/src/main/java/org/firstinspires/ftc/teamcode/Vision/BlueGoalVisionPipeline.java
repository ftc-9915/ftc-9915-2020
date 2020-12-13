/* Uses contouring to actively identify and draw a box around the largest ring.
Then uses the Cb channel value of the area to check for orange color and
the aspect ratio of the drawn box to build a "confidence value" from 0 to 1 for ring positions
FOUR and ONE and a hesitance value for ring position NONE. These confidence values
are then used to determine the most accurate Ring Position.

 */



package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.opencv.core.CvType.CV_64FC1;


/**
 * Vision Pipeline that uses contours to draw a rectangle around the largest orange object and uses the aspect ratio of the rectangle and cb color channel values of the contents of the
 * rectangle to determine confidence ratings for the one, four and none configuration, and determines the most likely ring configuration
 *
 */

public class BlueGoalVisionPipeline extends OpenCvPipeline {


    //Boundary Line (Only detects above this to eliminate field tape)
    public static final int BOUNDARY = 160;

    public static final double CAMERA_HEIGHT = 8.25; //camera height inches for distance calculation
    public static final double HIGH_GOAL_HEIGHT = 35.875; //camera height inches for distance calculation
    public static double distanceYToGoal = HIGH_GOAL_HEIGHT - CAMERA_HEIGHT;


    //Mask constants to isolate blue coloured subjects
    public Scalar lowerHSV = new Scalar(110, 0, 236);
    public Scalar upperHSV = new Scalar(121, 36, 255);

    //working mat variables
    public Mat HSVFrame = new Mat();
    public Mat MaskFrame = new Mat();
    public Mat ContourFrame = new Mat();
    public Mat RingAnalysisZone = new Mat();
    public Mat CbFrame = new Mat();


    //USEFUL VALUES TO BE ACCESSED FROM AUTONOMOUS
    public Rect maxRect = new Rect();
    public volatile double distanceXToGoal;
    public volatile double distanceToGoal;


    @Override
    public Mat processFrame(Mat input) {


        //define maxWidth in method so it resets every cycle
        int maxWidth = 0;

        //Convert to HSV color space
        Imgproc.cvtColor(input, HSVFrame, Imgproc.COLOR_RGB2HSV);

        //Mask Frame to only include tuned blue objects
        Core.inRange(HSVFrame, lowerHSV, upperHSV, MaskFrame);

        //Erode excess items in mask
        Imgproc.erode(MaskFrame, MaskFrame, new Mat(), new Point(-1, -1), 1);

        //create arraylist to populate with contour points
        ArrayList<MatOfPoint> contourList = new ArrayList<>();

        //find contours, outputs points to "contours", outputs image to "ContourFrame"
        Imgproc.findContours(MaskFrame, contourList, ContourFrame, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE); //RETR_Tree retrieves contour with layered hierarchy

        //traverse through contourList to find contour with largest width, which is presumed to be ring
        for(MatOfPoint contour : contourList){

            MatOfPoint2f convertedContour = new MatOfPoint2f(contour.toArray());

            //draw rectangle around contour
            Rect contourRect = Imgproc.boundingRect(convertedContour);

            //find largest contour rectangle in list
            if(contourRect.width > maxWidth && contourRect.y + contourRect.height < BOUNDARY){
                maxWidth = contourRect.width;
                maxRect = contourRect;
            }

            contour.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            convertedContour.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }
        

        //draw rectangle and report position
        Imgproc.rectangle(input, maxRect, new Scalar(0,255,0), 2);

        //calculate and set x distance in inches using calibration curve
        distanceXToGoal = 1/(0.000771 * maxRect.height) + 0.813229571984;

        //calculate distance using pythagorean theorem
        distanceToGoal = Math.sqrt(Math.pow(distanceXToGoal, 2) + Math.pow(distanceYToGoal, 2));


        return input;
    }




}
