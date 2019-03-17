/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;

import javax.swing.text.html.HTMLDocument.Iterator;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class TargetCamera implements Runnable, TargetInfo {
    Object lock = new Object();
    double distance;
    CvSink cvSink;
    CvSource outputStream;
    Mat input, source, output, mask, hierarchy;
    ArrayList<MatOfPoint> contours;
    Thread thread;
    double fieldOfView = 15 / 26.0;
    double imageWidth = 160;
    double angle = 0;
    /*
    double[] coefficients = new double[] {
        15.5887262581734, 38646.2278445553, -7.35691471763911e+6,
        8.80924154583584e8, -4.04032044493288e+10
    };
    */
    protected double[] coefficients = new double[] {
		20.075,
		30236.23,
		-2.4818e+6,
		1.1185e+8
	};
    public TargetCamera() {
        System.out.println("start camera thread");
        
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(320, 240);
      
        cvSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("Blur", 320, 240);
        input = new Mat();
        source = new Mat();
        output = new Mat();
        mask = new Mat();
        hierarchy = new Mat();
        contours = new ArrayList<MatOfPoint>();
        thread = new Thread(this);
    }

    public void start() {
        thread.start();
    }
    MatOfPoint2f contour2f = new MatOfPoint2f();
    ArrayList<RotatedRect> rightRects, leftRects;
    double IDEAL_RATIO = 5.5/2.0;
    ArrayList<double[]> targets;
    public void run() {
      try {
        Runtime.getRuntime().exec("/usr/bin/v4l2-ctl -d /dev/video0 --set-ctrl exposure_auto=1");
        Runtime.getRuntime().exec("/usr/bin/v4l2-ctl -d /dev/video0 --set-ctrl exposure_absolute=800");
      } catch(Exception ex) {
        ex.printStackTrace();
      }
          
        while(!Thread.interrupted()) {
          long start = System.currentTimeMillis();
          cvSink.grabFrame(input);
          if (input.empty()) {
            System.out.print("source empty");
            continue;
          }
          contours = new ArrayList<MatOfPoint>();
          Imgproc.resize(input, input, new Size(320, 240));
          Core.flip(input, source, -1);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          Imgproc.threshold(output, mask, 200, 255, Imgproc.THRESH_BINARY);
          Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
          long elapsed = System.currentTimeMillis() - start;
          //System.out.println(contours.size());
          rightRects = new ArrayList<RotatedRect>();
          leftRects = new ArrayList<RotatedRect>();
          double maxArea = 0;
          double[] targetInfo = null;
          for (int i=0; i < contours.size(); i++) {
            
            try {
              if (Imgproc.contourArea(contours.get(i)) < 60) continue;

              contours.get(i).convertTo(contour2f, CvType.CV_32F);
              RotatedRect ellipse = Imgproc.fitEllipseDirect(contour2f);
              //s += ellipse.angle + " ";
              Size size = ellipse.size;
              double aspect_ratio = size.height/size.width;
              if (aspect_ratio < 1) aspect_ratio = 1/aspect_ratio;
              if (aspect_ratio > IDEAL_RATIO * 1.5 ||
                  aspect_ratio < IDEAL_RATIO * 0.5) continue;
              //System.out.println(ellipse.angle);
              double angle = ellipse.angle;
              if (10 < angle && angle < 20) leftRects.add(ellipse);
              if (160 < angle && angle < 170) rightRects.add(ellipse);
            }
            catch(Exception ex) {
              ex.printStackTrace();
            }
          }
          //System.out.println(leftRects.size() + " " + rightRects.size());
          for (int j = 0; j < leftRects.size(); j++) {
            Point lcenter = leftRects.get(j).center;
            double shortest = 9999;
            RotatedRect right_match = null;
            for (int k = 0; k < rightRects.size(); k++) {
              Point rcenter = rightRects.get(k).center;
              if (rcenter.x < lcenter.x) continue;
              double dx = rcenter.x - lcenter.x;
              double dy = rcenter.y - lcenter.y;
              double distance = Math.sqrt(dx*dx+dy*dy);
              if (distance < shortest) {
                shortest = distance;
                right_match = rightRects.get(k);
              }
              
              //System.out.println(right_match);
              if (right_match != null) {
                Size lSize = leftRects.get(j).size;
                Size rSize = right_match.size;
                double area = (lSize.height * lSize.width + rSize.width * rSize.height)/2.0;
                double cx = (lcenter.x + right_match.center.x)/2.0;
                //System.out.println(cx);
                if (area > maxArea) {
                  targetInfo = new double[] {cx, area};
                  maxArea = area;
                }
              }
            }
          }
          if (targetInfo == null) {
            distance = Double.NaN;
            angle = Double.NaN;
            continue;
          }
          double area = targetInfo[1];
          double center = targetInfo[0] - imageWidth;
          //double[] targetInfo = Robot.targetInfo.getTargetInfo();
          double distance = coefficients[0];
          double x = 1/area;
          for (int i = 1; i < coefficients.length; i++) {
            distance += coefficients[i]*x;
            x /= area;
          }
          
          setDistance(distance);
          angle = -Math.toDegrees(Math.atan(fieldOfView / imageWidth * center));
          SmartDashboard.putNumber("distance", distance);
          SmartDashboard.putNumber("angle", angle);
          if(RobotMap.DEBUG){
            //System.out.println(/*elapsed + " " + */distance + " " + angle);
          }
          outputStream.putFrame(mask);
          //System.out.println(area + " " + center + " " + distance + " " + x + " " + angle);
          
        }
        
    }

    public double getDistance(){
            return distance;
    }
    public void setDistance(double d) {
            distance = d;
    }
    public double getAngle(){
            return angle;
    }
    public void setAngle(double a) {
            angle = a;
    }
}
