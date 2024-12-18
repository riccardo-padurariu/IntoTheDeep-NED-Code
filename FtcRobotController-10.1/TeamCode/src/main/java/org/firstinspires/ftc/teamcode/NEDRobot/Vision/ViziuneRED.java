package org.firstinspires.ftc.teamcode.NEDRobot.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ViziuneRED extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */


    public enum Side {
        LEFT,
        RIGHT
    }
    public int PortocalaMin=70;
    public int PortocalaMax=100;
    public int PiersicaMin=112;
    public int PiersicaMax=125;
    public int RulotaMin=126;
    public int RulotaMax=130;
    public static Side SIDE = Side.LEFT;

    public static int LEFTSIDE_REGION_X = 200;
    public static int LEFTSIDE_REGION_Y = 800;

    public static int CENTRAL_REGION_X = 1120;
    public static int CENTRAL_REGION_Y = 750;

    public static int RIGHTSIDE_REGION_X = 1900;
    public static int RIGHTSIDE_REGION_Y = 800;


    public static int REGION_WIDTH = 200;
    public static int REGION_HEIGHT = 200;

    public Point sleeve1_pointA =new Point(LEFTSIDE_REGION_X, LEFTSIDE_REGION_Y) ;
    public Point sleeve1_pointB =new Point(LEFTSIDE_REGION_X + REGION_WIDTH, LEFTSIDE_REGION_Y + REGION_HEIGHT) ;


    public Point sleeve2_pointA =new Point(CENTRAL_REGION_X, CENTRAL_REGION_Y) ;
    public Point sleeve2_pointB =new Point(CENTRAL_REGION_X + REGION_WIDTH, CENTRAL_REGION_Y + REGION_HEIGHT) ;


    public Point sleeve3_pointA =new Point(RIGHTSIDE_REGION_X, RIGHTSIDE_REGION_Y) ;
    public Point sleeve3_pointB =new Point(RIGHTSIDE_REGION_X + REGION_WIDTH, RIGHTSIDE_REGION_Y + REGION_HEIGHT) ;




    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Anchor point definitions


    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;
    private boolean isLeft;

    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    public int avg1;
    public int avg2;
    public int avg3;



    public ViziuneRED() {
        isLeft = (Side.LEFT == SIDE);
    }

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(sleeve1_pointA, sleeve1_pointB));
        region2_Cb = Cb.submat(new Rect(sleeve2_pointA, sleeve2_pointB));
        region3_Cb = Cb.submat(new Rect(sleeve3_pointA, sleeve3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];

        Imgproc.rectangle(
                input,
                sleeve1_pointA,
                sleeve1_pointB,
                YELLOW,
                2
        );

        Imgproc.rectangle(
                input,
                sleeve2_pointA,
                sleeve2_pointB,
                YELLOW,
                2
        );

        Imgproc.rectangle(
                input,
                sleeve3_pointA,
                sleeve3_pointB,
                YELLOW,
                2
        );
        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];

        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        if (max==avg2) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve2_pointA,
                    sleeve2_pointB,
                    YELLOW,
                    -1
            );

        } else if (max==avg3) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve3_pointA,
                    sleeve3_pointB,
                    YELLOW,
                    -1
            );

        } else if(max==avg1) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve1_pointA,
                    sleeve1_pointB,
                    YELLOW,
                    -1
            );
            };


        return input;
    }

    // Returns an enum being the current position where the robot will park
    public int getPosition() {
        if(position== ParkingPosition.LEFT)
            return 1;
        else if (position== ParkingPosition.CENTER)
            return 2;
        return 3;
    }

    public int getAnalysis()
    {
        return avg1;
    }

}