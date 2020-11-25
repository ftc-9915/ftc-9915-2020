package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
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

public class VisionPipelineDynamic extends OpenCvPipeline {

    /**
     * An enum to define the ring position
     */
    public enum RingPosition
    {
        FOUR,
        ONE,
        NONE
    }



    //Cb Threshhold Values
    private static final int FOUR_RING_CB_AVERAGE = 157;
    private static final int FOUR_RING_ASPECT_RATIO = 43/39;

    private static final int ONE_RING_CB_AVERAGE = 146;
    private static final double ONE_RING_ASPECT_RATIO = 43/21;

    private static final double HESITANCE_THRESHOLD = 0.2;

    public static final double DIMENSION_WEIGHT = 1;
    public static final double COLOR_WEIGHT = 1;


    private Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    private Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

    //working variables
    private Mat YcrCbFrame = new Mat();
    private Mat MaskFrame = new Mat();
    private Mat ContourFrame = new Mat();
    private Mat RingAnalysisZone = new Mat();
    private Mat CbFrame = new Mat();

    private Rect maxRect;

    private int avgCbValue;
    public volatile RingPosition position;


    @Override
    public Mat processFrame(Mat input) {
        //define maxWidth in method so it resets every cycle
        int maxWidth = 0;

        //extract Cb color space
        Imgproc.cvtColor(input, YcrCbFrame, Imgproc.COLOR_RGB2YCrCb);

        //apply mask to isolate orange rings
        Core.inRange(YcrCbFrame, lowerOrange, upperOrange, MaskFrame);

        //apply blur to remove extraneous results
        Imgproc.GaussianBlur(MaskFrame, MaskFrame, new Size(5.0, 15.0), 0.00);

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
            if(contourRect.width > maxWidth){
                maxWidth = contourRect.width;
                maxRect = contourRect;
            }

            contour.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            convertedContour.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }

        Imgproc.rectangle(input, maxRect, new Scalar(0,0,255), 2);

        Core.extractChannel(YcrCbFrame, CbFrame, 1);
        RingAnalysisZone = CbFrame.submat(maxRect);

        //find average cb value of the submat
        avgCbValue = (int) Core.mean(RingAnalysisZone).val[0];

        //compute four ring error and one ring confidence
        double fourRingCbError = (double) Math.abs(FOUR_RING_CB_AVERAGE - avgCbValue) / FOUR_RING_CB_AVERAGE;
        double fourRingDimensionError = Math.abs(FOUR_RING_ASPECT_RATIO  - ((double) maxRect.width / maxRect.height));
        double fourRingConfidence = 1 - ((fourRingCbError * COLOR_WEIGHT) + (fourRingDimensionError * DIMENSION_WEIGHT));
        telemetry.addData( "fourRingCbError",  fourRingCbError );
        telemetry.addData( "fourRingDimensionError",  fourRingDimensionError );
        telemetry.addData( "fourRingConfidence",  fourRingConfidence );

        //compute one ring error and one ring confidence
        double oneRingCbError = (double) Math.abs(ONE_RING_CB_AVERAGE - avgCbValue) / ONE_RING_CB_AVERAGE;
        double oneRingDimensionError = Math.abs(ONE_RING_ASPECT_RATIO  - ((double) maxRect.width / maxRect.height));
        double oneRingConfidence = 1 - ((oneRingCbError * COLOR_WEIGHT) + (oneRingDimensionError * DIMENSION_WEIGHT));
        telemetry.addData( "oneRingCbError",  oneRingCbError);
        telemetry.addData( "oneRingDimensionError",  oneRingDimensionError );
        telemetry.addData( "oneRingConfidence",  oneRingConfidence );




        //compute hesitance, difference between smallest ring confidence and 1
        double hesitance = 1 - Math.max(oneRingConfidence, fourRingConfidence);
        telemetry.addData( "hesitance",  hesitance );
        telemetry.update();

        if (hesitance > HESITANCE_THRESHOLD){
            position = RingPosition.NONE;
        } else if (oneRingConfidence > fourRingConfidence){
            position = RingPosition.ONE;
        } else {
            position = RingPosition.FOUR;
        }


        Imgproc.rectangle(input, maxRect, new Scalar(0,255,0), 2);
        Imgproc.putText(input, String.valueOf(position) , new Point(maxRect.x, maxRect.y - 20), Imgproc.FONT_HERSHEY_PLAIN, 0.5, new Scalar(0,255,0), 1);


        return input;
    }


}
