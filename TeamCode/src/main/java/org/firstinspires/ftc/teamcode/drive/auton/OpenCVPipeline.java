package org.firstinspires.ftc.teamcode.drive.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class OpenCVPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    List<MatOfPoint> contoursList = new ArrayList<>();

    public static int x1 = 100;
    public static int y1 = 35;
    public static int x2 = 160;
    public static int y2 = 75;

    public enum Location {
        DUCK,
        NO_DUCK

    }
    private Location location;
    static final Rect test_ROI = new Rect(
            new Point(x1,y1),
            new Point(x2,y2)
    );
    static final Rect middle_ROI = new Rect(
            new Point(100,155),
            new Point(160,195)

    );
    static double PERCENT_COLOR_THRESHOLD = 0.05;
    public OpenCVPipeline(Telemetry t) {telemetry = t;}

    int REALDUCKLocation = 0;

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
        Imgproc.putText(input,"Test",new Point(100,100),0,1,new Scalar(255,0,0));
        Scalar lowHSV = new Scalar(23,50,70);
        Scalar highHSV = new Scalar(32,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);
        List<MatOfPoint> countersList = new ArrayList<>();
        Imgproc.findContours(mat, countersList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        // Imgproc.drawContours(mat, countersList,0, new Scalar(255,0,0));
        Mat test = mat.submat(test_ROI);

        double testValue = Core.sumElems(test).val[0] / test_ROI.area()/255;

        test.release();

        telemetry.addData("Test raw value",Core.sumElems(test).val[0]);
        telemetry.addData("Test percentage",Core.sumElems(test).val[0]/test_ROI.area()/255);

        boolean duckTest = testValue > PERCENT_COLOR_THRESHOLD ; //If this is true, then there is a duck

        /* it only shows found if it's in position one
        should we change this? move it somewhere else?
        */
        if(duckTest){
            location = Location.DUCK;
            telemetry.addData("Duck","Found!!!" );

        } else {
            location = Location.NO_DUCK;
            telemetry.addData("Duck","Missing!!!" );


        }
        telemetry.update();

        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

        Scalar noDuckFound = new Scalar(255,0,0);
        Scalar duckFound = new Scalar(0,255,0);
        Rect theRealDucky = new Rect(new Point(0,0), new Point(1,1));
        for (MatOfPoint countor : countersList)
        {

            Rect rect = Imgproc.boundingRect(countor);
            if (rect.area() > theRealDucky.area()) {
                theRealDucky = rect;
            }

        }
        RobotLog.d("DUCKY LOCATION: ", theRealDucky.toString());
        int center = theRealDucky.y + (theRealDucky.height/2);
        Imgproc.rectangle(mat, theRealDucky, location == Location.DUCK? duckFound:noDuckFound);
        Imgproc.line(mat, new Point(0, mat.height()/3), new Point(mat.width(), mat.height()/3), new Scalar(255,0,0));
        if (theRealDucky.area() > 1000) {
            if (center > (2 * mat.height()/3)) {
                REALDUCKLocation = 3;
            }
            else if (center < (2 * mat.height()/3)) {
                REALDUCKLocation = 2;
            }
        }else{
            REALDUCKLocation = 1;
        }
//        telemetry.addData("Rectangle Area", theRealDucky.area());
//        telemetry.addData("Duck Position", REALDUCKLocation);
        return mat;
    }
    public Location getLocation() {
        return location;

    }

    /**
     *
     * @return Duck Location
     * 1: Duck on Left
     * 2: Duck on Center
     * 3: Duck on Right
     */
    public int getDuckLocation() {
        return REALDUCKLocation;
    }
}
