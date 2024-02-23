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
    /** Boolean that represents whether or not vision pose estimation should be enabled or disabled entirely. If this is disabled, none of the following settings will have any impact.*/
    public static final Boolean kVisionEnabled = false;
    
    /** Boolean that represents whether or not Hardware In The Loop (using physical hardware in a simulation) should be used. */
    public static final Boolean kPhysicalSimulation = false;
    /** Boolean that represents whether or not extra vision debug information (specifically creating a Field2d for each camera) should be enabled. This can be intensive, so when not in use, it should be disabled. */
    public static final Boolean kExtraVisionDebug = false;
    /** Boolean that represents whether or not to use vision pose estimation in a simulation. This will allow testing of the vision subsystem, but will also make the testing of other robot components annoying, as the vision pose isn't always 100% right (and is also potentially slow when being simulated). */
    public static final Boolean kVisionPoseEstimationSimulation = false;
    public static final AprilTagFieldLayout kFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Transform3d kRobotToLeftCam =
        new Transform3d(new Translation3d(0.5, 0.5, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d kRobotToRightCam =
        new Transform3d(new Translation3d(0.5, -0.5, 0), new Rotation3d(0, 0, 0));
  }
  
  public static final class MotorConstants {
    public static final int leftArmMotorID = 13; //1
    public static final int rightArmMotorID = 14; //2
    public static final int leftElevatorMotorID = 15; //3
    public static final int rightElevatorMotorID = 16; //4

    public static final int leftPrimerMotorID = 17; //5
    public static final int rightPrimerMotorID = 18; //6
    public static final int leftShooterMotorID = 19; //7
    public static final int rightShooterMotorID = 20; //8

    public static final int intakeMotorID = 7; //9
  }

  public static class SpeedConstants {
    public static final double kIntake = 0.6;
    public static final double kPrime = 0.5;
    public static final double kRetract = -0.1;
    public static final double kShooter = 0.885;
    public static final double kOutake = -kIntake;
  }
}
