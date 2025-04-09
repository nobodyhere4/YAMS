package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Optional;

public class TalonFXSWrapper extends SmartMotorController
{

  /**
   * {@link TalonFXS} motor controller
   */
  private final TalonFXS                      talonfxs;
  /**
   * {@link DCMotor} controlled by {@link TalonFXS}
   */
  private final DCMotor                       motor;
  /**
   * Configurator
   */
  private final TalonFXSConfigurator          configurator;
  /**
   * {@link DCMotorSim} for the {@link TalonFXS}.
   */
  private       Optional<DCMotorSim>          talonfxsSim      = Optional.empty();
  /**
   * Configuration of the motor
   */
  private       TalonFXSConfiguration         talonfxConfig;
  /**
   * Mechanism position in rotations.
   */
  private       StatusSignal<Angle>           mechanismPosition;
  /**
   * Mechanism velocity in rotations per second.
   */
  private       StatusSignal<AngularVelocity> mechanismVelocity;
  /**
   * Supply current of the motor controller.
   */
  private       StatusSignal<Current>         supplyCurrent;
  /**
   * Stator current of the motor controller.
   */
  private       StatusSignal<Current>         statorCurrent;
  /**
   * DutyCycle of the motor controller.
   */
  private       StatusSignal<Double>          dutyCycle;
  /**
   * The motor voltage.
   */
  private       StatusSignal<Voltage>         outputVoltage;
  /**
   * Rotor position.
   */
  private       StatusSignal<Angle>           rotorPosition;
  /**
   * Rotor velocity.
   */
  private       StatusSignal<AngularVelocity> rotorVelocity;
  /**
   * Temperature status
   */
  private       StatusSignal<Temperature>     deviceTemperature;
  /**
   * Setpoint angle for the closed loop controller.
   */
  private       Optional<Angle>               setpointPosition = Optional.empty();

  /**
   * Create the {@link TalonFXS} wrapper
   *
   * @param controller  {@link TalonFXS}
   * @param motor       {@link DCMotor}
   * @param smartConfig {@link SmartMotorControllerConfig}
   */
  public TalonFXSWrapper(TalonFXS controller, DCMotor motor, SmartMotorControllerConfig smartConfig)
  {
    this.talonfxs = controller;
    this.motor = motor;
    this.config = smartConfig;
    configurator = talonfxs.getConfigurator();
    talonfxConfig = new TalonFXSConfiguration();
    mechanismPosition = talonfxs.getPosition();
    mechanismVelocity = talonfxs.getVelocity();
    dutyCycle = talonfxs.getDutyCycle();
    statorCurrent = talonfxs.getStatorCurrent();
    supplyCurrent = talonfxs.getSupplyCurrent();
    outputVoltage = talonfxs.getMotorVoltage();
    rotorPosition = talonfxs.getRotorPosition();
    rotorVelocity = talonfxs.getRotorVelocity();
    deviceTemperature = talonfxs.getDeviceTemp();
    setupSimulation();
    applyConfig(smartConfig);
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      talonfxsSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(motor,
                                                                                  0.001,
                                                                                  config.getGearing()
                                                                                        .getRotorToMechanismRatio()),
                                               motor));
    }
  }

  @Override
  public void seedRelativeEncoder()
  {

  }

  @Override
  public void synchronizeRelativeEncoder()
  {
    if (config.getFeedbackSynchronizationThreshold().isPresent())
    {
//      if (sparkAbsoluteEncoder.isPresent())
//      {
//        if (!Rotations.of(sparkRelativeEncoder.getPosition()).isNear(Rotations.of(sparkAbsoluteEncoder.get()
//                                                                                                      .getPosition()),
//                                                                     config.getFeedbackSynchronizationThreshold()
//                                                                           .get()))
//        {
//          seedRelativeEncoder();
//        }
//      }
    }
  }

  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {

  }

  @Override
  public void simIterate()
  {

  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
    talonfxsSim.ifPresent(sim -> sim.setAngularVelocity(velocity.in(RadiansPerSecond)));
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    setEncoderVelocity(config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    // TODO: Handle absolute encoders here.
    talonfxs.setPosition(angle);
    talonfxsSim.ifPresent(dcMotorSim -> dcMotorSim.setAngle(angle.in(Radians)));
  }

  @Override
  public void setEncoderPosition(Distance distance)
  {
    setEncoderPosition(config.convertToMechanism(distance));
  }

  @Override
  public void setPosition(Angle angle)
  {
    setpointPosition = angle == null ? Optional.empty() : Optional.of(angle);

  }

  @Override
  public void setPosition(Distance distance)
  {
    setPosition(config.convertToMechanism(distance));
  }

  @Override
  public void setVelocity(LinearVelocity velocity)
  {
    setVelocity(config.convertToMechanism(velocity));
  }

  @Override
  public void setVelocity(AngularVelocity angle)
  {

  }

  @Override
  public double getDutyCycle()
  {
    return dutyCycle.refresh().getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    talonfxs.set(dutyCycle);
  }


  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    return false;
  }

  @Override
  public Current getSupplyCurrent()
  {
    return supplyCurrent.refresh().getValue();
  }

  @Override
  public Current getStatorCurrent()
  {
    return statorCurrent.refresh().getValue();
  }

  @Override
  public Voltage getVoltage()
  {
    return outputVoltage.refresh().getValue();
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    talonfxs.setVoltage(voltage.in(Volts));
  }

  @Override
  public DCMotor getDCMotor()
  {
    return motor;
  }

  @Override
  public LinearVelocity getMeasurementVelocity()
  {
    return config.convertFromMechanism(getMechanismVelocity());
  }

  @Override
  public Distance getMeasurementPosition()
  {
    return config.convertFromMechanism(getMechanismPosition());
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    return mechanismVelocity.refresh().getValue();
  }

  @Override
  public Angle getMechanismPosition()
  {
    return mechanismPosition.refresh().getValue();
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return rotorVelocity.refresh().getValue();
  }

  @Override
  public Angle getRotorPosition()
  {
    return rotorPosition.refresh().getValue();
  }

  @Override
  public Temperature getTemperature()
  {
    return deviceTemperature.refresh().getValue();
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return config;
  }

  @Override
  public Optional<Angle> getMechanismSetpoint()
  {
    return setpointPosition;
  }
}
