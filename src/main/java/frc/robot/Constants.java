// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /** Constants for the VisionSubsystem of the robot. */
  public static class VisionConstants {
    /** Boolean that represents whether or not Hardware In The Loop (using physical hardware in a simulation) should be used. */
    public static final Boolean kPhysicalSimulation = false;
    /** Boolean that represents whether or not extra vision debug information (specifically creating a Field2d for each camera) should be enabled. This can be intensive, so when not in use, it should be disabled. */
    public static final Boolean kExtraVisionDebug = false;
    public static final AprilTagFieldLayout kFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Transform3d kRobotToLeftCam =
        new Transform3d(new Translation3d(0.5, 0.5, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d kRobotToRightCam =
        new Transform3d(new Translation3d(0.5, -0.5, 0), new Rotation3d(0, 0, 0));
  }
}
