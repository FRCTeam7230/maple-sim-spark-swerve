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

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static class LimelightConstants
  {
    public static final String kLimelightName = "limelight";

    public static final double kDriveForwardKp = 2.4;
    public static final double kDriveHorizontalKp = 1.5; //output = -1 to 1, .1 m off want .1 m/s, 0.1 m/s = 0.04 % output, .04 = kp*0.1, kp = .4
    public static final double kRotationKp = 0.04; //output = -1 to 1, 15 deg off want 60 deg/sec, 60 deg/sec = 1.0 % output, 1.0 = kp*15deg, kp = 0.06

    public static final double kHorizontalOffset = 0.17;
    public static final double kForwardExtendedOffset = -0.43; //TODO: -0.47 at comp!
    public static final double kForwardUnextendedOffset = -0.7;

    public static final double kPositionErrorThreshold = 0.05;
    public static final double kRotationErrorThreshold = 1; // deg
    public static final double kElevatorTolerance = 0.02; //m

    public enum reefAlignSide
    {
      Right,
      Left,
    }
  }
    public static class L1Constants {
      public static final int kL1Motor = 4;//Choose a port for motor. 
  
      // L1 TODO - Will need an intake position, a score position, a stow position, and an L4 score position
      public static final double intakePosition  = 21;
      public static final double scorePosition   = 342; // deg
      public static final double stowPosition    = 24; // deg
      public static final double l4ScorePosition = 255; // deg     was 239
      public static final double l4Intake        = 24;
  
      public static final double kElevMaxHeightForL4Scoring = .5;
  
      public static final double kScoreForwardOffset = -1; //TODO: convert 5.125 inches to m (plus distance to LL???)
  
      //public static final double kResetCurrent = 0; //max current tbd TODO: Will need to tune these currents
      public static final int kMaxCurrent   = 20;//In amps? TODO: Will need to tune these currents
  
      public static final double kL1Kp = 0.015; //TODO: Will need to tune this, I lowered it to start 
      public static final double kL1Ki = 0;
      public static final double kL1Kd = 0;
      public static final double kL1RampRate = 0.2;//not sure what this one is for
  
      //Note, all should be ok at zero except kG 
      public static final double kL1kS = 0;
      public static final double kL1kV = 0;
      public static final double kL1kG = 0.6;
      public static final double kL1kA = 0;
  
      public static final double elevatorHeight = 0.5; //This is the constant to determine whether to extend or retract based on elevator height.
      public static final double overrideHeight = 0.6;
      public static final double overrideHeightDown = 0.7;
  
  //     Gear ratio
  // Output pulley radius/diameter
  // Encoder ticks per revolution
  // CAN IDs
  // Will need to decide on a motor current limit config value
  
    }
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class ElevatorConstants
    {
  
      public static final double kElevatorKp = 5;
      public static final double kElevatorKi = 0;
      public static final double kElevatorKd = 0;
  
      public static final double kElevatorkS = 0.0; // volts (V)
      public static final double kElevatorkG = 0.762; // volts (V)
      public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
      public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  
      public static final double kElevatorGearing = 10.0;
      public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
      public static final double kCarriageMass = 4.0; // kg
  
      // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
      public static final double kMinElevatorHeightMeters = 0.0;
      //public static final double kMaxElevatorHeightMeters = 10.25;
      public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72);
  
      public static final double kRotationToMeters = kElevatorDrumRadius * 2 * Math.PI;
      public static final double kRPMtoMPS = (kElevatorDrumRadius * 2 * Math.PI) / 60;
      public static final double kElevatorMaxVelocity = 3.5;
      public static final double kElevatorMaxAcceleration = 2.5;
    }

    public static class IntakeConstants {
      public static enum Side {
        BLUE,
        RED
      }
      //Intake position
      //
    }
  }
