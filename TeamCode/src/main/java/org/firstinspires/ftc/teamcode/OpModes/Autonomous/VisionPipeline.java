package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tests.EasyOpenCVExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {

    //Global Constants
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    //Global Working Variables
    Mat RingMat = new Mat();
    Mat GoalMat = new Mat();

    //Ring Pipeline Constants
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

    static final int REGION_WIDTH = 35;
    static final int REGION_HEIGHT = 25;

    final int FOUR_RING_THRESHOLD = 150;
    final int ONE_RING_THRESHOLD = 135;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);



    //Enum for ring configurations
    public enum RingPosition {
        FOUR,
        ONE,
        NONE
    }

    //Ring Pipeline Working Variables
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;


    //State
    OpenCvCamera camera;
    Telemetry telemetry;
    RingPosition ringPosition;
    Point highGoalLocation;




    //constructor requires Telemetry object for reporting
    public VisionPipeline(OpenCvCamera camera, Telemetry telemetry) {
        this.camera = camera;
        this.telemetry = telemetry;
    }

    //Converts frame from input from RGB to YCrCb color space
    // and extracts Cb channel to "Cb" variable
    public void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }


    @Override
    public void init(Mat firstFrame)
    {
        //converts initial frame
        inputToCb(firstFrame);

        //Create rectangular submat to analyze
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        if(avg1 > FOUR_RING_THRESHOLD){
            ringPosition = RingPosition.FOUR;
        }else if (avg1 > ONE_RING_THRESHOLD){
            ringPosition = RingPosition.ONE;
        }else{
            ringPosition = RingPosition.NONE;
        }

        telemetry.addLine("")

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1); // Negative thickness means solid fill

        return input;
    }





}