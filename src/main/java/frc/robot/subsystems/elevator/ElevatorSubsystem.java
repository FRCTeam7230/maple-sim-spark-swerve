// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
public class ElevatorSubsystem extends SubsystemBase
{

  // This gearbox represents a gearbox containing 1 Neo
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
  // private final DCMotor m_L1SubsystemGearbox = DCMotor.getNEO(1);
  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  private final SparkMax                  m_motor      = new SparkMax(1, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder           m_encoder    = m_motor.getEncoder();
  private final SparkMaxSim               m_motorSim   = new SparkMaxSim(m_motor, m_elevatorGearbox);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);



  // ArmFeedforward m_L1feedforward = new ArmFeedforward(
  //       Constants.L1Constants.kL1kS, //volts 
  //       Constants.L1Constants.kL1kG, //volts,  test this using revclient 
  //       Constants.L1Constants.kL1kV, //volts * seconds / radians
  //       Constants.L1Constants.kL1kA  //volts * seconds ^ 2 / radians
  //       );//In case we need this if L1 needs to be more accurate, smooth
  // private final SparkMax                  m_L1motor = new SparkMax(Constants.L1Constants.kL1Motor, MotorType.kBrushed);
  // private final SparkMaxConfig            m_L1Config = new SparkMaxConfig();
  //   private final AbsoluteEncoder           m_L1encoder = m_L1motor.getAbsoluteEncoder();
  //   private final SparkClosedLoopController m_L1controller  = m_L1motor.getClosedLoopController();
  // private final SparkMaxSim               m_L1motorSim   = new SparkMaxSim(m_L1motor, m_L1SubsystemGearbox);

  // private final SingleJointedArmSim m_L1SubsystemSim = 
  //     new SingleJointedArmSim(
  //       m_L1SubsystemGearbox, 
  //       2,
  //       0.5,
  //       8,
  //       -Math.PI/2,
  //       Math.PI/2,
  //       true,
  //       0,
  //       0,0);
  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d         m_mech2d         = new Mechanism2d(20, 6);
  private final MechanismRoot2d     m_mech2dRoot     = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 80));
  // private final Mechanism2d         m_L1Subsystem2d = new Mechanism2d(20, 6);
  // private final MechanismRoot2d     m_L1mech2dRoot     = m_L1Subsystem2d.getRoot("L1 Subsystem Root", 10, 5);
  // private final MechanismLigament2d m_L1SubsystemMech =
  //         m_L1mech2dRoot.append(
  //             new MechanismLigament2d("wrist", 8, m_L1SubsystemSim.getAngleRads(), 6, new Color8Bit(Color.kPurple)));
  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder
        .positionConversionFactor(ElevatorConstants.kRotationToMeters) // Converts Rotations to Meters
        .velocityConversionFactor(ElevatorConstants.kRPMtoMPS); // Converts RPM to MPS
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
        .maxMotion
        .maxVelocity(ElevatorConstants.kElevatorMaxVelocity)
        .maxAcceleration(ElevatorConstants.kElevatorMaxAcceleration)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.01);
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


    // m_L1Config.absoluteEncoder
    //   .inverted(true)
    //   .positionConversionFactor(360)
    //   .velocityConversionFactor(360);
    //   // .setSparkMaxDataPortConfig(); //i think this configures it to the encoder through the controller, but i'm not sure.

    //   m_L1Config.closedLoop
    //   .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //Maybe this is how you do it? 
    //   .pid(Constants.L1Constants.kL1Kp,Constants.L1Constants.kL1Ki,Constants.L1Constants.kL1Kd)//I don't understand the kSlot stuff. What do each of the slots represent. A: It represents pid settings that can be stored in each "slot".
    //   .outputRange(-0.3,0.3)//determines the speed limit. L1 TODO - this will likely need to be raised before tuning
    //   .positionWrappingEnabled(true)
    //   .positionWrappingInputRange(-180.0, 180.0)
    //   .maxMotion
    //   .maxAcceleration(0)
    //   .maxVelocity(0) //I saw in the documentation that this is getting replaced with cruiseVelocity
    //   .allowedClosedLoopError(5);//TODO: Tune this L1 TODO - is this in degrees? It is, so 10 degrees for now.
      
      
    //   m_L1Config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    //   m_L1Config.smartCurrentLimit(Constants.L1Constants.kMaxCurrent);

    //   m_L1motor.configure(m_L1Config, ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
   
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    ///SmartDashboard.putNumber("Elevator Encoder", m_encoder.getPosition());
    // SmartDashboard.putData("L1Subsystem Sim", m_L1Subsystem2d);
  }

  /**
   * Advance the simulation.
   */
  public void simulationPeriodic()
  {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    SmartDashboard.putNumber("Elevator Encoder", m_encoder.getPosition());
    

    // m_L1SubsystemSim.setInput(m_L1motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // // Next, we update it. The standard loop time is 20ms.
    // m_L1SubsystemSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery voltage
    // m_L1motorSim.iterate(m_L1SubsystemSim.getVelocityRadPerSec(),RoboRioSim.getVInVoltage(), 0.020);

    // // SimBattery estimates loaded battery voltages
    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(m_L1SubsystemSim.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    m_controller.setReference(goal,
                              ControlType.kPosition,
                              ClosedLoopSlot.kSlot0,
                              m_feedforward.calculate(m_encoder.getVelocity()));
  }
  // public void reachL1Goal(double goal)
  // {
  //   m_L1controller.setReference(goal,
  //                             ControlType.kPosition,
  //                             ClosedLoopSlot.kSlot0,
  //                             m_L1feedforward.calculate(m_L1encoder.getPosition(),m_L1encoder.getVelocity()));
  // }


  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeight()
  {
    return m_encoder.getPosition();
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeight(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }
  // public Command setL1Goal(double goal)
  // {
  //   return run(() -> reachL1Goal(goal));
  // }
  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor.set(0.0);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry()
  {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(RobotBase.isSimulation() ? m_elevatorSim.getPositionMeters() : m_encoder.getPosition());
    // m_L1SubsystemMech.setAngle(m_L1SubsystemSim.getAngleRads()*180/Math.PI);
  }

  @Override
  public void periodic()
  {
    updateTelemetry();
  }
}