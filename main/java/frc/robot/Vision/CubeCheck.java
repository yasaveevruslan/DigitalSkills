package frc.robot.Vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

// import frc.robot.RobotContainer;

public class CubeCheck
{
    public static int Check(Mat orig, boolean down)
    {
        int y = down ? 270 : 170;
        Mat rotated = Viscad.RotateImage(orig, 90);

        Mat cut = Viscad.ExtractImage(rotated, new Rect(240, y, 160, 160));
        Mat hsv = new Mat();
        Imgproc.cvtColor(cut, hsv, Imgproc.COLOR_BGR2HSV);
        int result = CheckInside(hsv);
        rotated.release();
        cut.release();
        hsv.release();
        return result;
    }

    public static int CheckYellow(Mat orig)
    {
        Mat rotated = Viscad.RotateImage(orig, 90);

        Mat cut = Viscad.ExtractImage(rotated, new Rect(220, 310, 160, 160));
        Mat hsv = new Mat();
        Imgproc.cvtColor(cut, hsv, Imgproc.COLOR_BGR2HSV);

        Mat threshBlue = Viscad.Threshold(hsv, new Point(0, 70), new Point(40, 200), new Point(60, 220));
        int blueArea = Viscad.ImageTrueArea(threshBlue);
        if (blueArea > 6000)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    private static int CheckInside(Mat img)
    {
        Mat threshBlue = Viscad.Threshold(img, new Point(60, 140), new Point(120, 240), new Point(70, 180));
        int blueArea = Viscad.ImageTrueArea(threshBlue);
        if (blueArea > 300)
        {
            threshBlue.release();
            Mat threshBlueRed = Viscad.Threshold(img, new Point(0, 220), new Point(120, 190), new Point(130, 180));
            if (Viscad.ImageTrueArea(threshBlueRed) > 300)
            {                
                threshBlueRed.release();
                return 1;
            }
            else
            {
                threshBlueRed.release();
                return 0;
            }
        }
        else
        {
            threshBlue.release();
            Mat threshWhiteRed = Viscad.Threshold(img, new Point(140, 220), new Point(100, 255), new Point(60, 170));
            Mat threshWhiteRed2 = Viscad.Threshold(img, new Point(0, 16), new Point(100, 255), new Point(60, 170));
            if (Viscad.ImageTrueArea(Viscad.BinaryOr(threshWhiteRed, threshWhiteRed2)) > 300)
            {
                threshWhiteRed.release();
                threshWhiteRed2.release();
                return 2;
            }
            else
            {
                threshWhiteRed.release();
                threshWhiteRed2.release();
                return 0;
            }
        }
    }
}