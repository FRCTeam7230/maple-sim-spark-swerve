package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class L1Subsystem extends SubsystemBase {
    private final DCMotor m_L1SubsystemGearbox = DCMotor.getNEO(1);
    ArmFeedforward m_L1feedforward = new ArmFeedforward(
        Constants.L1Constants.kL1kS, //volts 
        Constants.L1Constants.kL1kG, //volts,  test this using revclient 
        Constants.L1Constants.kL1kV, //volts * seconds / radians
        Constants.L1Constants.kL1kA  //volts * seconds ^ 2 / radians
        );//In case we need this if L1 needs to be more accurate, smooth
  private final SparkMax                  m_L1motor = new SparkMax(Constants.L1Constants.kL1Motor, MotorType.kBrushed);
  private final SparkMaxConfig            m_L1Config = new SparkMaxConfig();
    private final AbsoluteEncoder           m_L1encoder = m_L1motor.getAbsoluteEncoder();
    //private final RelativeEncoder           m_L1encoder    = m_L1motor.getEncoder();
   //private final SparkAbsoluteEncoderSim = new SparkAbsoluteEncoderSim(m_L1motor);
    private final SparkClosedLoopController m_L1controller  = m_L1motor.getClosedLoopController();
  private final SparkMaxSim               m_L1motorSim   = new SparkMaxSim(m_L1motor, m_L1SubsystemGearbox);
  //private final SparkAbsoluteEncoderSim m_L1encoder;
  //private final SparkRelativeEncoderSim
  
  private final SingleJointedArmSim m_L1SubsystemSim = 
      new SingleJointedArmSim(
        m_L1SubsystemGearbox, 
        2,
        0.5,
        0.5,
        Integer.MIN_VALUE,
        Integer.MAX_VALUE,
        true,
        0,
        0.01,0);
  private final Mechanism2d         m_L1Subsystem2d = new Mechanism2d(20, 6);
  private final MechanismRoot2d     m_L1mech2dRoot     = m_L1Subsystem2d.getRoot("L1 Subsystem Root", 10, 5);
  private final MechanismLigament2d m_L1SubsystemMech =
          m_L1mech2dRoot.append(
              new MechanismLigament2d("wrist", 0.5, m_L1SubsystemSim.getAngleRads(), 6, new Color8Bit(Color.kPurple)));

    public L1Subsystem () {
        
        m_L1Config.absoluteEncoder//absolute
      //m_L1Config.encoder//relative
      .inverted(true)//since it returns its own class, it can chain.
      .positionConversionFactor(360)
      .velocityConversionFactor(360);
      //.setSparkMaxDataPortConfig(); //i think this configures it to the encoder through the controller, but i'm not sure.

      m_L1Config.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //Change this when chaging from absolute to relative
      .pid(Constants.L1Constants.kL1Kp,Constants.L1Constants.kL1Ki,Constants.L1Constants.kL1Kd)//I don't understand the kSlot stuff. What do each of the slots represent. A: It represents pid settings that can be stored in each "slot".
      .outputRange(-0.3,0.3)//determines the speed limit. L1 TODO - this will likely need to be raised before tuning
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(-180.0, 180.0)
      .maxMotion
      .maxAcceleration(0)
      .maxVelocity(0) //I saw in the documentation that this is getting replaced with cruiseVelocity
      .allowedClosedLoopError(5);//TODO: Tune this L1 TODO - is this in degrees? It is, so 10 degrees for now.
      
    // m_L1Config.closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
    //     .maxMotion
    //     .maxVelocity(ElevatorConstants.kElevatorMaxVelocity)
    //     .maxAcceleration(ElevatorConstants.kElevatorMaxAcceleration)
    //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
    //     .allowedClosedLoopError(0.01);
   // m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      //m_L1encoder.setPositionConversionFactor(360);//Can't chain because this is a void method
      //m_L1encoder.setVelocityConversionFactor(360);

      m_L1Config.idleMode(SparkBaseConfig.IdleMode.kBrake);
      m_L1Config.smartCurrentLimit(Constants.L1Constants.kMaxCurrent);

      m_L1motor.configure(m_L1Config, ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
      
      SmartDashboard.putData("L1Subsystem Sim", m_L1Subsystem2d);
      
    }   
    public void simulationPeriodic()
  {
    m_L1SubsystemSim.setInput(m_L1motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_L1SubsystemSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_L1motorSim.iterate(m_L1SubsystemSim.getVelocityRadPerSec(),RoboRioSim.getVInVoltage(), 0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_L1SubsystemSim.getCurrentDrawAmps()));


        SmartDashboard.putNumber("L1 Encoder", m_L1encoder.getPosition());
      SmartDashboard.putNumber("L1 Encoder Velociy", m_L1encoder.getVelocity());
  }
    public void reachL1Goal(double goal)
  {
    m_L1controller.setReference(goal,
                              ControlType.kPosition,
                              ClosedLoopSlot.kSlot0,
                              0);
        SmartDashboard.putNumber("L1 Goal Position", goal);
                              //m_L1feedforward.calculate(m_L1SubsystemSim.getAngleRads()*Math.PI/180,m_L1encoder.getVelocity()*Math.PI/180));
    
    }     
  public Command setL1Goal(double goal)
  {
    return run(() -> reachL1Goal(goal));
  }
  public void updateTelemetry()
  {
    // Update elevator visualization with position
    m_L1SubsystemMech.setAngle(m_L1SubsystemSim.getAngleRads()*180/Math.PI);
  }
  public void getSetPoint(){

  }
  @Override
  public void periodic()
  {
    updateTelemetry();
  }
}
