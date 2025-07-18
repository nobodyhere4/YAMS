package yams.telemetry;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import java.util.Optional;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SmartMotorControllerTelemetry
{


  /**
   * {@link TelemetryVerbosity} for the {@link SmartMotorController}
   */
  private final TelemetryVerbosity                  verbosity           = TelemetryVerbosity.HIGH;
  /**
   * Mechanism lower limit reached.
   */
  public        boolean                             mechanismLowerLimit = false;
  /**
   * Mechanism upper limit reached.
   */
  public        boolean                             mechanismUpperLimit = false;
  /**
   * Motor temperature cutoff reached.
   */
  public        boolean                             temperatureLimit    = false;
  /**
   * Velocity PID controller used.
   */
  public        boolean                             velocityControl     = false;
  /**
   * Elevator feedforward used.
   */
  public        boolean                             elevatorFeedforward = false;
  /**
   * Arm feedforward used.
   */
  public        boolean                             armFeedforward      = false;
  /**
   * Simple feedforward used.
   */
  public        boolean                             simpleFeedforward   = false;
  /**
   * Motion profiling used.
   */
  public        boolean                             motionProfile       = false;
  /**
   * Setpoint position given.
   */
  public        double                              setpointPosition    = 0;
  /**
   * Setpoint velocity given.
   */
  public        double                              setpointVelocity    = 0;
  /**
   * Feedforward voltage supplied to the {@link SmartMotorController}
   */
  public        double                              feedforwardVoltage  = 0.0;
  /**
   * PID Output voltage supplied to the {@link SmartMotorController}
   */
  public        double                              pidOutputVoltage    = 0.0;
  /**
   * Output voltage to the {@link SmartMotorController}
   */
  public        double                              outputVoltage       = 0.0;
  /**
   * Stator current (motor controller output current) to the Motor.
   */
  public        double                              statorCurrent       = 0.0;
  /**
   * Motor temperature.
   */
  public        Temperature                         temperature         = Fahrenheit.of(72);
  /**
   * Mechanism distance.
   */
  public        Distance                            distance            = Meters.of(0);
  /**
   * Mechanism linear velocity.
   */
  public        LinearVelocity                      linearVelocity      = MetersPerSecond.of(0);
  /**
   * Mechanism position.
   */
  public        Angle                               mechanismPosition;
  /**
   * Mechanism velocity.
   */
  public        AngularVelocity                     mechanismVelocity;
  /**
   * Rotor position.
   */
  public        Angle                               rotorPosition;
  /**
   * Rotor velocity.
   */
  public        AngularVelocity                     rotorVelocity;
  /**
   * Network table to publish to.
   */
  private       NetworkTable                        dataNetworkTable;
  /**
   * Network table for tuning.
   */
  private       NetworkTable                        tuningNetworkTable;
  /**
   * Mechanism lower limit boolean publisher
   */
  private       BooleanPublisher                    mechanismLowerLimitPublisher;
  /**
   * Mechanism upper limit boolean publisher
   */
  private       BooleanPublisher                    mechanismUpperLimitPublisher;
  /**
   * Motor temperature limit hit.
   */
  private       BooleanPublisher                    temperatureLimitPublisher;
  /**
   * Velocity control used.
   */
  private       BooleanPublisher                    velocityControlPublisher;
  /**
   * Elevator feedforward used.
   */
  private       BooleanPublisher                    elevatorFeedforwardPublisher;
  /**
   * Arm feedforward used.
   */
  private       BooleanPublisher                    armFeedforwardPublisher;
  /**
   * Simple motor feedforward used.
   */
  private       BooleanPublisher                    simpleFeedforwardPublisher;
  /**
   * Motion profile used.
   */
  private       BooleanPublisher                    motionProfilePublisher;
  /**
   * Setpoint targetted.
   */
  private       DoublePublisher                     setpointPositionPublisher;
  /**
   * Setpoint velocity targetted.
   */
  private       DoublePublisher                     setpointVelocityPublisher;
  /**
   * Feedforward voltage output.
   */
  private       DoublePublisher                     feedforwardVoltagePublisher;
  /**
   * PID Output voltage.
   */
  private       DoublePublisher                     pidOutputVoltagePublisher;
  /**
   * Motor controller output voltage.
   */
  private       DoublePublisher                     outputVoltagePublisher;
  /**
   * Stator current output.
   */
  private       DoublePublisher                     statorCurrentPublisher;
  /**
   * Motor temperature
   */
  private       DoublePublisher                     temperaturePublisher;
  /**
   * Distance/Mechanism measurement
   */
  private       DoublePublisher                     measurementPositionPublisher;
  /**
   * Linear Velocity/Mechanism measurement velocity.
   */
  private       DoublePublisher                     measurementVelocityPublisher;
  /**
   * Mechanism position
   */
  private       DoublePublisher                     mechanismPositionPublisher;
  /**
   * Mechanism velocity
   */
  private       DoublePublisher                     mechanismVelocityPublisher;
  /**
   * Rotor position
   */
  private       DoublePublisher                     rotorPositionPublisher;
  /**
   * Rotor velocity.
   */
  private       DoublePublisher                     rotorVelocityPublisher;
  /**
   * Telemetry config
   */
  private       SmartMotorControllerTelemetryConfig config              = new SmartMotorControllerTelemetryConfig();

  /**
   * Setup Telemetry Pub/Sub fields.
   *
   * @param publishTable {@link NetworkTable} that holds all of the output fields.
   * @param tuningTable  {@link NetworkTable} that holds all of the tuning fields.
   * @param verbosity    {@link TelemetryVerbosity} to apply.
   */
  public void setupTelemetry(NetworkTable publishTable, NetworkTable tuningTable, TelemetryVerbosity verbosity)
  {
    if (!publishTable.equals(this.dataNetworkTable))
    {
      config = new SmartMotorControllerTelemetryConfig().withTelemetryVerbosity(verbosity);
      dataNetworkTable = publishTable;
      mechanismLowerLimitPublisher = dataNetworkTable.getBooleanTopic("Mechanism Lower Limit").publish();
      mechanismUpperLimitPublisher = dataNetworkTable.getBooleanTopic("Mechanism Upper Limit").publish();
      temperatureLimitPublisher = dataNetworkTable.getBooleanTopic("Temperature Limit").publish();
      velocityControlPublisher = dataNetworkTable.getBooleanTopic("Velocity Control").publish();
      elevatorFeedforwardPublisher = dataNetworkTable.getBooleanTopic("Elevator Feedforward").publish();
      armFeedforwardPublisher = dataNetworkTable.getBooleanTopic("Arm Feedforward").publish();
      simpleFeedforwardPublisher = dataNetworkTable.getBooleanTopic("Simple Feedforward").publish();
      motionProfilePublisher = dataNetworkTable.getBooleanTopic("Motion Profile").publish();
      setpointPositionPublisher = dataNetworkTable.getDoubleTopic("Setpoint Position (Rotations)").publish();
      setpointVelocityPublisher = dataNetworkTable.getDoubleTopic("Setpoint Velocity (Rotations per Second)").publish();
      feedforwardVoltagePublisher = dataNetworkTable.getDoubleTopic("Feedforward Voltage").publish();
      pidOutputVoltagePublisher = dataNetworkTable.getDoubleTopic("PID Output (Voltage)").publish();
      outputVoltagePublisher = dataNetworkTable.getDoubleTopic("Motor Output Voltage").publish();
      statorCurrentPublisher = dataNetworkTable.getDoubleTopic("Stator Current (Amps)").publish();
      temperaturePublisher = dataNetworkTable.getDoubleTopic("Temperature (Celsius)").publish();
      measurementPositionPublisher = dataNetworkTable.getDoubleTopic("Measurement Position (Meters)").publish();
      measurementVelocityPublisher = dataNetworkTable.getDoubleTopic("Measurement Velocity (Meters per Second)")
                                                     .publish();
      mechanismPositionPublisher = dataNetworkTable.getDoubleTopic("Mechanism Position (Rotations)").publish();
      mechanismVelocityPublisher = dataNetworkTable.getDoubleTopic("Mechanism Velocity (Rotations per Second)")
                                                   .publish();
      rotorPositionPublisher = dataNetworkTable.getDoubleTopic("Rotor Position (Rotations)").publish();
      rotorVelocityPublisher = dataNetworkTable.getDoubleTopic("Rotor Velocity (Rotations per Second)").publish();
    }
  }

  /**
   * Setup Telemetry Pub/Sub fields.
   *
   * @param publishTable {@link NetworkTable} that holds all of the output fields.
   * @param tuningTable  {@link NetworkTable} that holds all of the tuning fields.
   * @param config       {@link SmartMotorControllerTelemetryConfig} to apply.
   */
  public void setupTelemetry(NetworkTable publishTable, NetworkTable tuningTable,
                             SmartMotorControllerTelemetryConfig config)
  {
    if (!publishTable.equals(this.dataNetworkTable))
    {
      this.config = config;
      dataNetworkTable = publishTable;
      if (config.mechanismLowerLimitEnabled)
      {
        mechanismLowerLimitPublisher = dataNetworkTable.getBooleanTopic("Mechanism Lower Limit").publish();
      }
      if (config.mechanismUpperLimitEnabled)
      {
        mechanismUpperLimitPublisher = dataNetworkTable.getBooleanTopic("Mechanism Upper Limit").publish();
      }
      if (config.temperatureLimitEnabled)
      {
        temperatureLimitPublisher = dataNetworkTable.getBooleanTopic("Temperature Limit").publish();
      }
      if (config.velocityControlEnabled)
      {
        velocityControlPublisher = dataNetworkTable.getBooleanTopic("Velocity Control").publish();
      }
      if (config.elevatorFeedforwardEnabled)
      {
        elevatorFeedforwardPublisher = dataNetworkTable.getBooleanTopic("Elevator Feedforward").publish();
      }
      if (config.armFeedforwardEnabled)
      {
        armFeedforwardPublisher = dataNetworkTable.getBooleanTopic("Arm Feedforward").publish();
      }
      if (config.simpleFeedforwardEnabled)
      {
        simpleFeedforwardPublisher = dataNetworkTable.getBooleanTopic("Simple Feedforward").publish();
      }
      if (config.motionProfileEnabled)
      {
        motionProfilePublisher = dataNetworkTable.getBooleanTopic("Motion Profile").publish();
      }
      if (config.setpointPositionEnabled)
      {
        setpointPositionPublisher = dataNetworkTable.getDoubleTopic("Setpoint Position (Rotations)").publish();
      }
      if (config.setpointVelocityEnabled)
      {
        setpointVelocityPublisher = dataNetworkTable.getDoubleTopic("Setpoint Velocity (Rotations per Second)")
                                                    .publish();
      }
      if (config.feedforwardVoltageEnabled)
      {
        feedforwardVoltagePublisher = dataNetworkTable.getDoubleTopic("Feedforward Voltage").publish();
      }
      if (config.pidOutputVoltageEnabled)
      {
        pidOutputVoltagePublisher = dataNetworkTable.getDoubleTopic("PID Output (Voltage)").publish();
      }
      if (config.outputVoltageEnabled)
      {
        outputVoltagePublisher = dataNetworkTable.getDoubleTopic("Motor Output Voltage").publish();
      }
      if (config.statorCurrentEnabled)
      {
        statorCurrentPublisher = dataNetworkTable.getDoubleTopic("Stator Current (Amps)").publish();
      }
      if (config.temperatureEnabled)
      {
        temperaturePublisher = dataNetworkTable.getDoubleTopic("Temperature (Celsius)").publish();
      }
      if (config.distanceEnabled)
      {
        measurementPositionPublisher = dataNetworkTable.getDoubleTopic("Measurement Position (Meters)").publish();
      }
      if (config.linearVelocityEnabled)
      {
        measurementVelocityPublisher = dataNetworkTable.getDoubleTopic("Measurement Velocity (Meters per Second)")
                                                       .publish();
      }
      if (config.mechanismPositionEnabled)
      {
        mechanismPositionPublisher = dataNetworkTable.getDoubleTopic("Mechanism Position (Rotations)").publish();
      }
      if (config.mechanismVelocityEnabled)
      {
        mechanismVelocityPublisher = dataNetworkTable.getDoubleTopic("Mechanism Velocity (Rotations per Second)")
                                                     .publish();
      }
      if (config.rotorPositionEnabled)
      {
        rotorPositionPublisher = dataNetworkTable.getDoubleTopic("Rotor Position (Rotations)").publish();
      }
      if (config.rotorVelocityEnabled)
      {
        rotorVelocityPublisher = dataNetworkTable.getDoubleTopic("Rotor Velocity (Rotations per Second)").publish();
      }
    }
  }

  /**
   * Publish {@link SmartMotorController} telemetry to {@link NetworkTable}
   *
   * @param publishTable {@link NetworkTable} to publish to.
   * @param verbosity    {@link TelemetryVerbosity} to publish.
   */
  public void publish()
  {

    if (dataNetworkTable != null)
    {
      mechanismLowerLimitPublisher.set(mechanismLowerLimit);
      mechanismUpperLimitPublisher.set(mechanismUpperLimit);
      temperatureLimitPublisher.set(temperatureLimit);
      velocityControlPublisher.set(velocityControl);
      elevatorFeedforwardPublisher.set(elevatorFeedforward);
      armFeedforwardPublisher.set(armFeedforward);
      simpleFeedforwardPublisher.set(simpleFeedforward);
      motionProfilePublisher.set(motionProfile);
      setpointPositionPublisher.set(setpointPosition);
      setpointVelocityPublisher.set(setpointVelocity);
      feedforwardVoltagePublisher.set(feedforwardVoltage);
      pidOutputVoltagePublisher.set(pidOutputVoltage);
      outputVoltagePublisher.set(outputVoltage);
      statorCurrentPublisher.set(statorCurrent);
      temperaturePublisher.set(temperature.in(Celsius));
      measurementPositionPublisher.set(distance.in(Meters));
      measurementVelocityPublisher.set(linearVelocity.in(MetersPerSecond));
      mechanismPositionPublisher.set(mechanismPosition.in(Rotations));
      mechanismVelocityPublisher.set(mechanismVelocity.in(RotationsPerSecond));
      rotorPositionPublisher.set(rotorPosition.in(Rotations));
      rotorVelocityPublisher.set(rotorVelocity.in(RotationsPerSecond));
    }
  }

  /**
   * Publish {@link SmartMotorController} telemetry to {@link NetworkTable} using a given
   * {@link SmartMotorControllerTelemetryConfig}
   *
   * @param publishTable {@link NetworkTable} to publish to.
   * @param config       {@link SmartMotorControllerTelemetryConfig} to publish from.
   */
  public void publishFromConfig(NetworkTable publishTable, SmartMotorControllerTelemetryConfig config)
  {

    if (dataNetworkTable != null)
    {
      if (config.mechanismLowerLimitEnabled)
      {
        mechanismLowerLimitPublisher.set(mechanismLowerLimit);
      }
      if (config.mechanismUpperLimitEnabled)
      {
        mechanismUpperLimitPublisher.set(mechanismUpperLimit);
      }
      if (config.temperatureLimitEnabled)
      {
        temperatureLimitPublisher.set(temperatureLimit);
      }
      if (config.velocityControlEnabled)
      {
        velocityControlPublisher.set(velocityControl);
      }
      if (config.elevatorFeedforwardEnabled)
      {
        elevatorFeedforwardPublisher.set(elevatorFeedforward);
      }
      if (config.armFeedforwardEnabled)
      {
        armFeedforwardPublisher.set(armFeedforward);
      }
      if (config.simpleFeedforwardEnabled)
      {
        simpleFeedforwardPublisher.set(simpleFeedforward);
      }
      if (config.motionProfileEnabled)
      {
        motionProfilePublisher.set(motionProfile);
      }
      if (config.setpointPositionEnabled)
      {
        setpointPositionPublisher.set(setpointPosition);
      }
      if (config.setpointVelocityEnabled)
      {
        setpointVelocityPublisher.set(setpointVelocity);
      }
      if (config.feedforwardVoltageEnabled)
      {
        feedforwardVoltagePublisher.set(feedforwardVoltage);
      }
      if (config.pidOutputVoltageEnabled)
      {
        pidOutputVoltagePublisher.set(pidOutputVoltage);
      }
      if (config.outputVoltageEnabled)
      {
        outputVoltagePublisher.set(outputVoltage);
      }
      if (config.statorCurrentEnabled)
      {
        statorCurrentPublisher.set(statorCurrent);
      }
      if (config.temperatureEnabled)
      {
        temperaturePublisher.set(temperature.in(Celsius));
      }
      if (config.distanceEnabled)
      {
        measurementPositionPublisher.set(distance.in(Meters));
      }
      if (config.linearVelocityEnabled)
      {
        measurementVelocityPublisher.set(linearVelocity.in(MetersPerSecond));
      }
      if (config.mechanismPositionEnabled)
      {
        mechanismPositionPublisher.set(mechanismPosition.in(Rotations));
      }
      if (config.mechanismVelocityEnabled)
      {
        mechanismVelocityPublisher.set(mechanismVelocity.in(RotationsPerSecond));
      }
      if (config.rotorPositionEnabled)
      {
        rotorPositionPublisher.set(rotorPosition.in(Rotations));
      }
      if (config.rotorVelocityEnabled)
      {
        rotorVelocityPublisher.set(rotorVelocity.in(RotationsPerSecond));
      }
    }
  }

  /**
   * Refresh the telemetry with data from the {@link SmartMotorController}. {@link #mechanismLowerLimit}
   * {@link #mechanismUpperLimit} {@link #temperatureLimit} {@link #distance} and {@link #linearVelocity} are updated
   * when appropriate {@link SmartMotorControllerConfig} options are set.
   *
   * @param smartMotorController The {@link SmartMotorController} to get data from.
   */
  public void refresh(SmartMotorController smartMotorController)
  {
    SmartMotorControllerConfig cfg = smartMotorController.getConfig();
    temperature = smartMotorController.getTemperature();
    statorCurrent = smartMotorController.getStatorCurrent().in(Amps);
    mechanismPosition = smartMotorController.getMechanismPosition();
    mechanismVelocity = smartMotorController.getMechanismVelocity();
    rotorPosition = smartMotorController.getRotorPosition();
    rotorVelocity = smartMotorController.getRotorVelocity();
    cfg.getMechanismLowerLimit().ifPresent(limit -> mechanismLowerLimit = mechanismPosition.lte(limit));
    cfg.getMechanismUpperLimit().ifPresent(limit -> mechanismUpperLimit = mechanismPosition.gte(limit));
    cfg.getTemperatureCutoff().ifPresent(limit -> temperatureLimit = temperature.gte(limit));
    cfg.getMechanismCircumference().ifPresent(cricumference -> {
      distance = smartMotorController.getMeasurementPosition();
      linearVelocity = smartMotorController.getMeasurementVelocity();
    });
  }

  // TODO: Add docs.
  protected enum BooleanTelemetryField
  {
    MechanismUpperLimit("limits/Mechanism Upper Limit", false),
    MechanismLowerLimit("limits/Mechanism Lower Limit", false),
    TemperatureLimit("limits/Temperature Limit", false),
    VelocityControl("control/Velocity Control", false),
    ElevatorFeedForward("control/Elevator Feedforward", false),
    ArmFeedForward("control/Arm Feedforward", false),
    SimpleMotorFeedForward("control/Simple Motor Feedforward", false),
    MotionProfile("control/Motion Profile", false);

    private final boolean currentValue;
    private final String  key;

    BooleanTelemetryField(String fieldName, Boolean defaultValue)
    {
      key = fieldName;
      currentValue = defaultValue;
    }

    public BooleanTelemetry create()
    {
      return new BooleanTelemetry(key, currentValue);
    }


  }

  protected enum DoubleTelemetryField
  {
    SetpointPosition("Setpoint Position", 0),
    SetpointVelocity("Setpoint Velocity", 0),
    FeedforwardVoltage("Feedforward voltage", 0),
    PIDOutputVoltage("PID Output voltage", 0),
    OutputVoltage("Output Voltage", 0),
    StatorCurrent("Stator Current", 0),
    SupplyCurrent("Supply Current", 0),
    MotorTemperature("Motor Temperature", 0),
    MeasurementPosition("Measurement Position", 0),
    MeasurementVelocity("Measurement Velocity", 0),
    MechanismPosition("Mechanism Position", 0),
    MechanismVelocity("Mechanism Velocity", 0),
    RotorPosition("Rotor Position", 0),
    RotorVelocity("Rotor Velocity", 0);

    private final DoublePublisher            publisher  = null;
    private final Optional<DoubleSubscriber> subscriber = Optional.empty();
    private final double                     currentValue;
    private final String                     key;

    DoubleTelemetryField(String fieldName, double defaultValue)
    {
      key = fieldName;
      currentValue = defaultValue;
    }

    public DoubleTelemetry create()
    {
      return new DoubleTelemetry(key, currentValue);
    }

  }
}
