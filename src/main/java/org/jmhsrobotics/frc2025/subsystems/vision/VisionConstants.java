// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  // TODO: Confirm and Identify which camera is on which side
  public static String camera0Name = "Blackbird";
  public static String camera1Name = "Overture";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // TODO: Make sure camera transformations are correctx
  //   public static Transform3d robotToCamera0 =
  //       new Transform3d(
  //           Units.inchesToMeters(8),
  //           Units.inchesToMeters(11),
  //           0.185,
  //           new Rotation3d(0.0, Units.degreesToRadians(-10), Units.degreesToRadians(-35)));

  //   public static Transform3d robotToCamera1 =
  //       new Transform3d(
  //           Units.inchesToMeters(8.5),
  //           Units.inchesToMeters(-11),
  //           0.185,
  //           new Rotation3d(0.0, Units.degreesToRadians(-10), Units.degreesToRadians(35)));

  public static Pose3d blackbirdCalibration =
      new Pose3d(
        //   0.824, 0.187, -0.028, new Rotation3d(new Quaternion(0.275, -0.052, 0.042, -0.959)));
    0.824, 0.187, -0.028, new Rotation3d());
  public static Pose3d overtureCalibration =new Pose3d();
    //   new Pose3d(
        //   0.795, -0.224, -0.139, new Rotation3d(new Quaternion(-.306, -0.119, -0.019, -0.944)));
    // 0.795, -0.224, -0.139, new Rotation3d());
    // blackbirdCalibration = new Pose3d();
  public static Pose3d calibrationOffset = new Pose3d(1, 0, 0.25, new Rotation3d());
  public static final Transform3d blackbirdToRobot = blackbirdCalibration.minus(calibrationOffset);
  public static final Transform3d overtureToRobot = overtureCalibration.minus(calibrationOffset);
  public static final Transform3d robotToCamera0 = blackbirdToRobot.inverse();
  public static final Transform3d robotToCamera1 = overtureToRobot.inverse();
  

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.25; // Meters
  public static double angularStdDevBaseline = 100; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
