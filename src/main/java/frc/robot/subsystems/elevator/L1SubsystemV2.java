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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
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

public class L1SubsystemV2 extends SubsystemBase{
    private final DCMotor m_L1SubsystemGearbox = DCMotor.getNEO(1);
    private final SparkMax                  m_L1motor = new SparkMax(Constants.L1Constants.kL1Motor, MotorType.kBrushed);
    //private final SparkMaxConfig            m_L1Config = new SparkMaxConfig();
    private final RelativeEncoder           m_L1encoder = m_L1motor.getEncoder();
    private final SparkClosedLoopController m_L1controller  = m_L1motor.getClosedLoopController();
    private final SparkMaxSim               m_L1motorSim   = new SparkMaxSim(m_L1motor, m_L1SubsystemGearbox);
    private final SingleJointedArmSim m_L1SubsystemSim = 
      new SingleJointedArmSim(
        m_L1SubsystemGearbox, 
        1,
        0.1,
        0.5,
        Integer.MIN_VALUE,
        Integer.MAX_VALUE,
        false,//when enabling this, remember that my reachgoal motor power is not strong enough to counter gravity.
        0
        );


    private final Mechanism2d         m_L1Subsystem2d = new Mechanism2d(20, 6);
    private final MechanismRoot2d     m_L1mech2dRoot     = m_L1Subsystem2d.getRoot("L1 Subsystem Root", 10, 5);
    private final MechanismLigament2d m_L1SubsystemMech =
            m_L1mech2dRoot.append(
                new MechanismLigament2d("wrist", 0.5, m_L1SubsystemSim.getAngleRads()*180/Math.PI, 6, new Color8Bit(Color.kPurple)));
    public L1SubsystemV2(){
        SparkMaxConfig m_L1Config = new SparkMaxConfig();
        //m_L1Config.encoder//relative
        //.inverted(true)//since it returns its own class, it can chain.
       // .positionConversionFactor(36000000)//This is doing nothing
        //.velocityConversionFactor(36000000);
        m_L1Config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //Change this when chaging from absolute to relative
        .pid(Constants.L1Constants.kL1Kp,Constants.L1Constants.kL1Ki,Constants.L1Constants.kL1Kd)//I don't understand the kSlot stuff. What do each of the slots represent. A: It represents pid settings that can be stored in each "slot".
        .outputRange(-0.01,0.01)//determines the speed limit. L1 TODO - this will likely need to be raised before tuning
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-180.0, 180.0)
        .maxMotion
        //.maxAcceleration(0)
        //.maxVelocity(0) //I saw in the documentation that this is getting replaced with cruiseVelocity
        .allowedClosedLoopError(5);//TODO: Tune this L1 TODO - is this in degrees? It is, so 10 degrees for now.
        m_L1motor.configure(m_L1Config, ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
        
        m_L1encoder.setPosition(0);
        SmartDashboard.putData("L1Subsystem Sim", m_L1Subsystem2d);
    }
    public void simulationPeriodic()
  {
  
    m_L1SubsystemSim.setInput(m_L1motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_L1SubsystemSim.update(0.020);
    m_L1motorSim.iterate(m_L1SubsystemSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.020);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_L1SubsystemSim.getCurrentDrawAmps()));

    m_L1encoder.setPosition(m_L1SubsystemSim.getAngleRads()*180/(Math.PI));//The lazy way out. Always makes the encoder equal to the actual angle. 
    //If you are stubborn to make this work, all I know is that the encoder value should be multiplied approximately by 10 to make it rotations.
    
    SmartDashboard.putNumber("L1 Encoder",m_L1encoder.getPosition());
    SmartDashboard.putNumber("L1 Encoder Velocity",m_L1encoder.getVelocity());
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain (degrees)
   */
  public void reachL1Goal(double goal)
  {
    m_L1controller.setReference(goal,
                              ControlType.kPosition,
                              ClosedLoopSlot.kSlot0,
                              0);
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
  @Override
  public void periodic()
  {
    updateTelemetry();
  }
}
