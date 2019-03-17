/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/**
 * Add your docs here.
 */
public class ReadCameraPorts {
    public static int readIRCamera() {
        try {
            Process p1 = Runtime.getRuntime().exec("/usr/bin/v4l2-ctl --list-devices");
            BufferedReader output = new BufferedReader(new InputStreamReader(p1.getInputStream()));
            String line = "";
            while ((line = output.readLine()) != null) {
                System.out.println(line);
                if (line.indexOf("ci_hdrc") >= 0) {
                    line = output.readLine();
                    System.out.println(" here:  " + line);
                    int index = line.indexOf("/dev/video") + "/dev/video".length();
                    if (index >= 0) return Integer.valueOf(line.substring(index, index+1));
                }
            }
        }
        catch(Exception ex) {
            System.out.println("Error reading camera ID");
        }
        return -1;
    }
}
