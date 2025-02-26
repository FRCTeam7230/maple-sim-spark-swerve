
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
      public static final double kScoreElevatorHeightMeters = Units.inchesToMeters(60);

  
      public static final double kRotationToMeters = kElevatorDrumRadius * 2 * Math.PI;
      public static final double kRPMtoMPS = (kElevatorDrumRadius * 2 * Math.PI) / 60;
      public static final double kElevatorMaxVelocity = 3.5;
      public static final double kElevatorMaxAcceleration = 2.5;
    }
    public static final class ElevatorSimConstants {
      public static final int kMotorPort = 0;
      public static final int kEncoderAChannel = 0;
      public static final int kEncoderBChannel = 1;
      public static final int kJoystickPort = 0;
    
      public static final double kElevatorKp = 0.75;
      public static final double kElevatorKi = 0;
      public static final double kElevatorKd = 0;
    
      public static final double kElevatorMaxV = 10.0; // volts (V)
      public static final double kElevatorkS = 0.0; // volts (V)
      public static final double kElevatorkG = 0.62; // volts (V)
      public static final double kElevatorkV = 3.9; // volts (V)
      public static final double kElevatorkA = 0.06; // volts (V)
    
      public static final double kElevatorGearing = 5.0;
      public static final double kElevatorDrumRadius = Units.inchesToMeters(1.0);
      public static final double kCarriageMass = Units.lbsToKilograms(12); // kg
    
      public static final double kSetpointMeters = Units.inchesToMeters(42.875);
      public static final double kLowerkSetpointMeters = Units.inchesToMeters(15);
      // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
      public static final double kMinElevatorHeightMeters = 0.0+0.03;//Idk why, but on the sim, the elevator is slightly afloat
      public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(50);
      public static final double kScoreElevatorHeightMeters = Units.inchesToMeters(40);
    
      // distance per pulse = (distance per revolution) / (pulses per revolution)
      //  = (Pi * D) / ppr
      public static final double kElevatorEncoderDistPerPulse =
          2.0 * Math.PI * kElevatorDrumRadius / 4096;
    }
}