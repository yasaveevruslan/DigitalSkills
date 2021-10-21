package frc.robot.Vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.*;

public class Viscad {
    public static Mat RotateImage(Mat src, double deg) {
        Mat dst = new Mat(src.rows(), src.cols(), src.type());
        Point src_center = new Point(src.cols() / 2.0F, src.rows() / 2.0F);
        Mat rot_mat = Imgproc.getRotationMatrix2D(src_center, 360 - deg, 1.0);
        Imgproc.warpAffine(src, dst, rot_mat, src.size());

        rot_mat.release();

        return dst;
    }

    public static Mat Threshold(Mat src, Point red, Point green, Point blue) {
        Mat outImage = new Mat();
        Core.inRange(src, new Scalar(red.x, green.x, blue.x), new Scalar(red.y, green.y, blue.y), outImage);

        return outImage;
    }

    public static Mat ThresholdGray(Mat src, Point range) {
        Mat outImage = new Mat();
        Core.inRange(src, new Scalar(range.x), new Scalar(range.y), outImage);

        return outImage;
    }

    public static Mat BinaryAnd(Mat first, Mat second) {
        Mat dst = new Mat();
        Core.bitwise_and(first, second, dst);
        return dst;
    }

    public static Mat BinaryOr(Mat first, Mat second) {
        Mat dst = new Mat();
        Core.bitwise_or(first, second, dst);
        return dst;
    }

    public static Mat BinaryNot(Mat src) {
        Mat dst = new Mat();
        Core.bitwise_not(src, dst);
        return dst;
    }

    public static Mat ResizeImage(Mat src, int w, int h) {
        Mat resizeImage = new Mat();
        Size sz = new Size(w, h);
        Imgproc.resize(src, resizeImage, sz);
        return resizeImage;
    }

    public static Mat ExtractImage(Mat src, Rect rect)
    {
        return src.submat(rect);
    }

    public static Mat Blur(Mat src, int power)
    {
        Mat dst = new Mat();
        Imgproc.blur(src, dst, new Size(power, power));
        return dst;
    }

    public static Mat RejectBorders(Mat src) {
        Mat output = new Mat();
        src.copyTo(output);
        int h = src.rows();
        int w = src.cols();

        for (int i = 0; i < h; ++i) {
            double[] temp = output.get(i, 0);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(0, i), new Scalar(0));
            }
            temp = output.get(i, w - 1);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(w - 1, i), new Scalar(0));
            }
        }
        for (int i = 0; i < w; ++i) {
            double[] temp = output.get(0, i);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(i, 0), new Scalar(0));
            }
            temp = output.get(h - 1, i);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(i, h - 1), new Scalar(0));
            }
        }
        return output;
    }

    public static Mat RejectBordersUp(Mat src) {
        Mat output = new Mat();
        src.copyTo(output);

        int h = src.rows();
        int w = src.cols();

        for (int i = 0; i < w; ++i) {
            double[] temp = output.get(0, i);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(i, 0), new Scalar(0));
            }
        }
        return output;
    }

    public static Mat RejectBordersUpFast(Mat src, int space) {
        Mat output = new Mat();
        src.copyTo(output);

        int h = src.rows();
        int w = src.cols();

        for (int i = 0; i + space < w; i += space) {
            double[] temp = output.get(0, i);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(i, 0), new Scalar(0));
            }
        }
        return output;
    }

    public static Mat RejectBordersWOBottomFast(Mat src, int space) {
        Mat output = new Mat();
        src.copyTo(output);
        int h = src.rows();
        int w = src.cols();

        for (int i = 0; i + space < h; i += space) {
            double[] temp = output.get(i, 0);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(0, i), new Scalar(0));
            }
            temp = output.get(i, w - 1);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(w - 1, i), new Scalar(0));
            }
        }
        for (int i = 0; i + space < w; i += space) {
            double[] temp = output.get(0, i);
            if (temp[0] == 255) {
                Imgproc.floodFill(output, new Mat(), new Point(i, 0), new Scalar(0));
            }
        }
        return output;
    }

    public static Mat ImageErdilCAD(Mat src, int power) {
        Mat dst = new Mat();
        src.copyTo(dst);

        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(2 * power + 1, 2 * power + 1));
        Imgproc.erode(src, dst, element);

        Mat dst2 = new Mat();
        dst.copyTo(dst2);

        Mat element1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(2 * power + 1, 2 * power + 1));
        Imgproc.dilate(dst, dst2, element1);

        dst.release();
        element.release();
        element1.release();

        return dst2;
    }

    public static Mat FindBiggestBlob(Mat inputImage) {

        Mat biggestBlob = inputImage.clone();

        double largest_area = 0;
        int largest_contour_index = 0;

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();

        Imgproc.findContours( biggestBlob, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        for( int i = 0; i< (int)contours.size(); i++ ) {
            double a = Imgproc.contourArea( contours.get(i) ,false);
            if( a > largest_area ){
                largest_area = a;
                largest_contour_index = i;
            }

        }

        Mat tempMat = biggestBlob.clone();
        Imgproc.drawContours( tempMat, contours, largest_contour_index, new Scalar(0), -1, 8, hierarchy );

        Mat subbed = new Mat();
        Core.subtract(biggestBlob, tempMat, subbed);
        return subbed;
    }

    public static Mat ParticleFilterCAD(Mat inputImage, final int minArea) {

        Mat biggestBlob = inputImage.clone();

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();

        Imgproc.findContours( biggestBlob, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        contours.removeIf(c -> Imgproc.contourArea( c ,false) < minArea);

        Mat tempMat = biggestBlob.clone();
        Imgproc.drawContours( tempMat, contours, -1, new Scalar(0), -1, 8 );

        Mat subbed = new Mat();
        Core.subtract(biggestBlob, tempMat, subbed);
        tempMat.release();
        biggestBlob.release();
        return subbed;
    }

    public static Mat Dilate(Mat src, int power)
    {
        Mat dst2 = new Mat();
        src.copyTo(dst2);

        Mat element1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(2 * power + 1, 2 * power + 1));
        Imgproc.dilate(src, dst2, element1);
        element1.release();

        return dst2;
    }

    public static List<Rect> ParticleAnalysis(Mat src, Mat dst)
    {
        Mat origCopy = src.clone();

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();

        Imgproc.findContours( origCopy, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        List<Rect> listOfBoxes = new ArrayList<>();
        List<Double> listOfAreas = new ArrayList<>();

        for (MatOfPoint cont : contours)
        {
            Rect bound = Imgproc.boundingRect(cont);
            listOfBoxes.add(bound);
            listOfAreas.add(Imgproc.contourArea(cont));
        }

        ArrayList<Rect> sortedList = new ArrayList(listOfBoxes);
        if (listOfBoxes.size() > 1)
            Collections.sort(sortedList, (left, right) -> ((int)listOfAreas.get(listOfBoxes.indexOf(right)).doubleValue() - (int)listOfAreas.get(listOfBoxes.indexOf(left)).doubleValue()));

        origCopy.release();
        hierarchy.release();

        return sortedList;
    }

    public static List<Rect> ParticleAnalysis(Mat src)
    {
        Mat deleteIt = new Mat();
        List<Rect> out = ParticleAnalysis(src, deleteIt);
        deleteIt.release();
        return out;
    }

    public static int ImageTrueArea(Mat src)
    {
        return Core.countNonZero(src);
    }

    public static int ImageTrueAreaContour(Mat src, Rect cont)
    {
        Mat imageROI = src.submat(cont);
        int c = ImageTrueArea(imageROI);
        imageROI.release();
        return c;
    }

    public static Point CenterOfMass(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);
        return new Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
    }

    public static Point CenterOfMass(Mat src) {
        Moments moments = Imgproc.moments(src);
        return new Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
    }

    public static Mat FillHolesCAD(Mat inputImage) {

        Mat biggestBlob = inputImage.clone();

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();

        Imgproc.findContours( biggestBlob, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
       for (MatOfPoint mop : contours)
        {
            Imgproc.fillConvexPoly(biggestBlob, mop, new Scalar(255));
        }
        hierarchy.release();
        return biggestBlob;
    }

    public static Mat AutoBrightnessCAD(Mat cut, Mat needed, int value, boolean outBGR)
    {
        Mat hsvCut = new Mat();
        Imgproc.cvtColor(cut, hsvCut, Imgproc.COLOR_BGR2HSV);
        ArrayList<Mat> dst = new ArrayList<>(3);
        Core.split(hsvCut, dst);

        MatOfDouble mu = new MatOfDouble();
        MatOfDouble sigma = new MatOfDouble();
        Core.meanStdDev(dst.get(2), mu, sigma);
        float mulK = (float)value / (float)mu.get(0,0)[0];

        Mat hsvOut = new Mat();
        Imgproc.cvtColor(needed, hsvOut, Imgproc.COLOR_BGR2HSV);
        ArrayList<Mat> dstOut = new ArrayList<>(3);
        Core.split(hsvOut, dstOut);
        Mat bright = new Mat();
        Core.multiply(dstOut.get(2), new Scalar(mulK), bright);

        Mat out = new Mat();
        Core.merge(new ArrayList<Mat>(Arrays.asList(new Mat[]{dstOut.get(0), dstOut.get(1), bright})), out);
        if (outBGR)
        {
            Mat newOut = new Mat();
            Imgproc.cvtColor(out, newOut, Imgproc.COLOR_HSV2BGR);
            return newOut;
        }
        return out;
    }

    public static List<MatOfPoint2f> RemakeContours(List<MatOfPoint> contours, float k)
    {
        List<MatOfPoint2f> newContours = new ArrayList<MatOfPoint2f>();
        for (MatOfPoint con : contours)
        {
            MatOfPoint2f c2f = new MatOfPoint2f(con.toArray());
            double conPerimeter = Imgproc.arcLength(c2f, true);
            float epsilon = k * (float)conPerimeter;
            MatOfPoint2f newCon = new MatOfPoint2f();
            Imgproc.approxPolyDP(c2f, newCon, epsilon, true);
            newContours.add(newCon);
            c2f.release();
        }
        return newContours;
    }
}