// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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
    public static final Boolean kPhysicalSimulation = true;
    /** Boolean that represents whether or not extra vision debug information (specifically creating a Field2d for each camera) should be enabled. This can be intensive, so when not in use, it should be disabled. */
    public static final Boolean kExtraVisionDebug = true;
    /** Boolean that represents whether or not to use vision pose estimation in a simulation. This will allow testing of the vision subsystem, but will also make the testing of other robot components annoying, as the vision pose isn't always 100% right (and is also potentially slow when being simulated). */
    public static final Boolean kVisionPoseEstimationSimulation = false;
    public static final AprilTagFieldLayout kFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Transform3d kRobotToLeftCam =
        new Transform3d(new Translation3d(Units.inchesToMeters(-15+2.75), Units.inchesToMeters(18.375), Units.inchesToMeters(7.34)), new Rotation3d(0, Units.degreesToRadians(46.5), 0));
    public static final Transform3d kRobotToRightCam =
        new Transform3d(new Translation3d(Units.inchesToMeters(15+2.75), Units.inchesToMeters(18.375),  Units.inchesToMeters(7.34)), new Rotation3d(0, Units.degreesToRadians(46.5), 0));
  }
  
  public static final class MotorConstants {
    public static final int leftArmMotorID = 13; //1  RUNNING ON CANIVORE
    public static final int rightArmMotorID = 14; //2  RUNNING ON CANIVORE
    public static final int leftElevatorMotorID = 15; //3  RUNNING ON CANIVORE
    public static final int rightElevatorMotorID = 16; //4  RUNNING ON CANIVORE

    public static final int leftPrimerMotorID = 17; //5  NOT ON CANIVORE
    public static final int rightPrimerMotorID = 18; //6  NOT ON CANIVORE
    public static final int leftShooterMotorID = 19; //7  NOT ON CANIVORE
    public static final int rightShooterMotorID = 20; //8  NOT ON CANIVORE

    public static final int intakeMotorID = 7; //9  NOT ON CANIVORE
  }

  public static class SpeedConstants {
    public static final double kIntake = 0.9;
    public static final double kPrime = 0.5;
    public static final double kRetract = -0.1;
    public static final double kShooter = 0.885;
    public static final double kOutake = -kIntake;
    public static final double kvF = 0;

  }

  public static class PhysicalConstants {
    public static final double armGearRatio = 0;
    public static final double primerGearRatio = 1;
    public static final double intakeGearRatio = 1;
    public static final double shooterGearRatio = 1;
    public static final double elevatorGearRatio = 25;

    public static final double shooterWheelRadiusMeters = Units.inchesToMeters(2);
  }

  public static class FieldConstants {
    public static final double kspeakerHeightMeters = Units.inchesToMeters(82.5-12);

  }
  
  public static class ShooterConstants  {
    public static final double kLeftP = 0.0005;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0.0008;
    public static final double kLeftIz = 0;
    public static final double kLeftFF = 0.0002;
    public static final double kLeftMax = 1;
    public static final double kLeftMin = -1;

    public static final double kRightP = 0.00045;
    public static final double kRightI = 0;
    public static final double kRightD = 0.0008;
    public static final double kRightIz = 0;
    public static final double kRightFF = 0.00018;
    public static final double kRightMax = 1;
    public static final double kRightMin = -1;
  }
}

