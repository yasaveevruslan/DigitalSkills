package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Vision.Viscad;


public class JavaCamera 
{
    private UsbCamera camera;
    private CvSink cvSink;
    private CvSource cv;
    private CvSource put;
    private CvSource put_1;
    private CvSource put_2;
    public static int nowTask = 2;
    public static int nowResult = 0;
    public DriveTrain tele;
    public static double blueArea;
    public static double whiteArea;
    public static double yellowArea;
    public CvSource board;
    public CvSource black;
    public CvSource test;


    public JavaCamera()
    {
        camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setFPS(30);
        cvSink = CameraServer.getInstance().getVideo();
        cv = CameraServer.getInstance().putVideo("blue", 640, 480);
        put = CameraServer.getInstance().putVideo("hsv", 640, 480);
        put_1 = CameraServer.getInstance().putVideo("white", 640, 480);
        put_2 = CameraServer.getInstance().putVideo("yellow", 640, 480);
        board = CameraServer.getInstance().putVideo("board", 640, 480);
        black = CameraServer.getInstance().putVideo("black", 640, 480);
        test = CameraServer.getInstance().putVideo("test", 640, 480);
    }

    public void Start(){
        new Thread(() -> {
            while(!Thread.interrupted()){
                Mat source = new Mat();
                if (cvSink.grabFrame(source) == 0) 
                {
                    continue;
                }
                if (source.cols() < 640 || source.rows() < 480)
                {
                    continue;
                }
                if (nowTask == 1){
                    Mat rot = new Mat();
                    rot = Viscad.ExtractImage(source, new Rect(220, 0, 200, 400));
                    Mat hsv = new Mat();
                    Imgproc.cvtColor(rot, hsv, Imgproc.COLOR_BGR2HSV);
                    put.putFrame(hsv);
                    Mat threshBlue = Viscad.Threshold(hsv, new Point(70, 130), new Point(100, 255), new Point(10, 150));
                    blueArea = Viscad.ImageTrueArea(threshBlue);
                    cv.putFrame(threshBlue);
                    Mat threshWhite = Viscad.Threshold(hsv, new Point(0, 255), new Point(150, 255), new Point(125, 200));
                    whiteArea = Viscad.ImageTrueArea(threshWhite);
                    put_1.putFrame(threshWhite);
                    Mat threshYellow = Viscad.Threshold(hsv, new Point(20, 70), new Point(150, 255), new Point(100, 200));
                    yellowArea = Viscad.ImageTrueArea(threshYellow);
                    put_2.putFrame(threshYellow);
                    if (blueArea > 3000){
                        nowResult = 3;
                    }else if(whiteArea > 1000 && whiteArea < 2000){
                        nowResult = 2;
                    }else if (yellowArea > 4000){
                        nowResult = 1;
                    }else{
                        nowResult = 0;
                    }
                }
            if (nowTask == 2){
                Mat labOut = new Mat();
                Imgproc.cvtColor(source, labOut, Imgproc.COLOR_BGR2Lab);
                ArrayList<Mat> layers = new ArrayList<>(3);
                Core.split(labOut, layers);
                Mat lFromLab = layers.get(0);
                Mat thresh = Viscad.ThresholdGray(lFromLab, new Point(80, 255));

                lFromLab.release();
                lFromLab = null;
                labOut.release();
                labOut = null;
                layers.clear();
                Mat rejected = Viscad.RejectBorders(thresh);
                thresh.release();
                thresh = null;
                Mat blurred = Viscad.Blur(rejected, 1);//3
                rejected.release();
                rejected = null;
        
                Mat dilated = Viscad.Dilate(blurred, 5);//7
                blurred.release();
                blurred = null;

                Mat threshDilate = Viscad.ThresholdGray(dilated, new Point(80, 255));
                dilated.release();
                dilated = null;
                List<MatOfPoint> c = new ArrayList<>();
                Mat newf = new Mat();
                Imgproc.findContours(threshDilate, c, newf, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                newf.release();
                black.putFrame(threshDilate);
                List<MatOfPoint2f> newContours = Viscad.RemakeContours(c, 0.052f);
                ArrayList<MatOfPoint2f> sortedList = new ArrayList(newContours);
                Collections.sort(sortedList, (left, right) -> ((int)Imgproc.contourArea(right) - (int)Imgproc.contourArea(left)));
                MatOfPoint2f biggest = sortedList.get(0);
                Moments moment = Imgproc.moments(biggest);
                int x = (int) (moment.get_m10() / moment.get_m00());
                int y = (int) (moment.get_m01() / moment.get_m00());

                Point[] sortedPoints = new Point[4];
                double[] data;
                int count = 0;
                for(int i=0; i<biggest.rows(); i++){
                    data = biggest.get(i, 0);
                    double datax = data[0];
                    double datay = data[1];
                    if(datax < x && datay < y){
                        sortedPoints[0]=new Point(datax,datay);
                        count++;
                    }else if(datax > x && datay < y){
                        sortedPoints[1]=new Point(datax,datay);
                        count++;
                    }else if (datax < x && datay > y){
                        sortedPoints[2]=new Point(datax,datay);
                        count++;
                    }else if (datax > x && datay > y){
                        sortedPoints[3]=new Point(datax,datay);
                        count++;
                    }
                }
                MatOfPoint2f src = new MatOfPoint2f(
                    sortedPoints[0],
                    sortedPoints[1],
                    sortedPoints[2],
                    sortedPoints[3]);
     
            MatOfPoint2f dst = new MatOfPoint2f(
                    new Point(0, 0),
                    new Point(450-1,0),
                    new Point(0,450-1),
                    new Point(450-1,450-1)      
                    );
                    Mat warpMat = Imgproc.getPerspectiveTransform(src,dst);
                    Mat destImage = new Mat();
                    Imgproc.warpPerspective(source, destImage, warpMat, source.size());
                    Mat bk = new Mat();
                    bk = Viscad.ExtractImage(destImage, new Rect(0, 0, 450, 450));
                    Mat resized = Viscad.ResizeImage(bk, 495, 495);
                    Mat ng = new Mat();
                    for (int i = 1; i <= 6; i++){
                        for (int j = 1; j <= 6; j ++){
                            ng = Viscad.ExtractImage(resized, new Rect(72 * j, 72 * i, 75, 75));
                        }
                    }
                    ng = Viscad.ExtractImage(resized, new Rect(72, 72, 75, 75));
                    board.putFrame(resized);
                    test.putFrame(ng);
                }
            source.release();
            }
        }).start();
    }
}