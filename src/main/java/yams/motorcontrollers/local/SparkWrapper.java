package yams.motorcontrollers.local;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.Optional;
import java.util.function.Supplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.telemetry.SmartMotorControllerTelemetry;

public class SparkWrapper implements SmartMotorController
{

  /**
   * Spark configuration retry count.
   */
  private final int                               CONFIG_RETRIES             = 4;
  /**
   * Spark configuration retry delay.
   */
  private final double                            CONFIG_RETRY_DELAY         = Milliseconds.of(5).in(Seconds);
  /**
   * Spark motor controller
   */
  private final SparkBase                         spark;
  /**
   * Spark base configuration.
   */
  private       SparkBaseConfig                   sparkBaseConfig;
  /**
   * Spark simulation.
   */
  private Optional<SparkSim>        sparkSim             = Optional.empty();
  /**
   * Spark relative encoder sim object.
   */
  private       Optional<SparkRelativeEncoderSim> sparkRelativeEncoderSim    = Optional.empty();
  /**
   * Spark relative encoder.
   */
  private       RelativeEncoder                   sparkRelativeEncoder;
  /**
   * Spark absolute encoder.
   */
  private Optional<AbsoluteEncoder> sparkAbsoluteEncoder = Optional.empty();
  /**
   * Spark absolute encoder sim object
   */
  private       Optional<SparkAbsoluteEncoderSim> sparkAbsoluteEncoderSim    = Optional.empty();
  /**
   * Motor type.
   */
  private final DCMotor                           motor;
  /**
   * {@link SmartMotorControllerConfig} for the motor.
   */
  private       SmartMotorControllerConfig        config;
  /**
   * Profiled PID controller for the motor controller.
   */
  private       Optional<ProfiledPIDController>   pidController              = Optional.empty();
  /**
   * Simple PID controller for the motor controller.
   */
  private       Optional<PIDController>           simplePidController        = Optional.empty();
  /**
   * Setpoint position
   */
  private Optional<Angle>           setpointPosition     = Optional.empty();
  /**
   * Setpoint velocity.
   */
  private Optional<AngularVelocity> setpointVelocity     = Optional.empty();
  /**
   * Telemetry.
   */
  private       SmartMotorControllerTelemetry     telemetry                  = new SmartMotorControllerTelemetry();
  /**
   * Thread of the closed loop controller.
   */
  private       Notifier                          closedLoopControllerThread = null;
  /**
   * Parent table for telemetry.
   */
  private Optional<NetworkTable>    parentTable          = Optional.empty();
  /**
   * {@link SmartMotorController} telemetry table.
   */
  private Optional<NetworkTable>    telemetryTable       = Optional.empty();


  /**
   * Create a {@link SmartMotorController} from {@link SparkMax} or {@link SparkFlex}
   *
   * @param controller {@link SparkMax} or {@link SparkFlex}
   * @param motor      {@link DCMotor} controller by the {@link SparkFlex} or {@link SparkMax}. Must be a brushless
   *                   motor.
   * @param config     {@link SmartMotorControllerConfig} to apply.
   */
  public SparkWrapper(SparkBase controller, DCMotor motor, SmartMotorControllerConfig config)
  {
    if (controller instanceof SparkMax)
    {
      sparkBaseConfig = new SparkMaxConfig();
    } else if (controller instanceof SparkFlex)
    {
      sparkBaseConfig = new SparkFlexConfig();
    } else
    {
      throw new IllegalArgumentException("Unsupported controller type: " + controller.getClass().getSimpleName());
    }

    this.motor = motor;
    spark = controller;
    sparkRelativeEncoder = controller.getEncoder();
    applyConfig(config);
    setupSimulation();

  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   * @return Successful configuration
   */
  private boolean configureSpark(Supplier<REVLibError> config)
  {
    for (int i = 0; i < CONFIG_RETRIES; i++)
    {
      if (config.get() == REVLibError.kOk)
      {
        return true;
      }
      Timer.delay(CONFIG_RETRY_DELAY);
    }
    return false;
  }

  /**
   * Setup the simulation for the {@link SparkBase} wrapper.
   */
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      sparkSim = Optional.of(new SparkSim(spark, motor));
      sparkRelativeEncoderSim = Optional.of(sparkSim.get().getRelativeEncoderSim());
    }
  }


  @Override
  public void seedRelativeEncoder()
  {
    if (sparkAbsoluteEncoder.isPresent())
    {
      sparkRelativeEncoder.setPosition(sparkAbsoluteEncoder.get().getPosition());
      sparkRelativeEncoderSim.ifPresent(sparkRelativeEncoderSim -> sparkRelativeEncoderSim.setPosition(
          sparkAbsoluteEncoder.get().getPosition()));
    }
  }

  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {
    if (RobotBase.isSimulation())
    {
      sparkSim.ifPresent(sim -> sim.iterate(mechanismVelocity.in(RotationsPerSecond),
                                            RoboRioSim.getVInVoltage(),
                                            config.getClosedLoopControlPeriod().in(Second)));
      sparkRelativeEncoderSim.ifPresent(sim -> sim.iterate(mechanismVelocity.in(RotationsPerSecond),
                                                           config.getClosedLoopControlPeriod().in(Seconds)));
      sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.iterate(mechanismVelocity.in(
          RotationsPerSecond), config.getClosedLoopControlPeriod().in(Seconds)));
    }
  }

  @Override
  public void simIterate()
  {
    if (RobotBase.isSimulation() && setpointVelocity.isPresent())
    {
      simIterate(setpointVelocity.get());
    }
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    setEncoderVelocity(config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    if (sparkAbsoluteEncoder.isPresent())
    {
      sparkBaseConfig.absoluteEncoder.zeroOffset(getMechanismPosition().minus(angle).in(Rotations));
      sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setPosition(angle.in(Rotations)));
    }
    sparkRelativeEncoder.setPosition(angle.in(Rotations));
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setPosition(angle.in(Rotations)));
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setVelocity(velocity.in(
        RotationsPerSecond)));
    sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setVelocity(velocity.in(
        RotationsPerSecond)));
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
    setpointVelocity = angle == null ? Optional.empty() : Optional.of(angle);
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {

    this.config = config;
    pidController = config.getClosedLoopController();

    // Handle simple pid vs profile pid controller.
    if (pidController.isEmpty())
    {
      simplePidController = config.getSimpleClosedLoopController();
      if (simplePidController.isEmpty())
      {
        throw new IllegalArgumentException("closed-loop controller must not be empty");
      }
    }

    // Handle closed loop controller thread
    if (closedLoopControllerThread == null)
    {
      closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);

    } else
    {
      closedLoopControllerThread.stop();
    }
    if (config.getTelemetryName().isPresent())
    {
      closedLoopControllerThread.setName(config.getTelemetryName().get());
    }
    if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
    {
      closedLoopControllerThread.startPeriodic(config.getClosedLoopControlPeriod().in(Second));
    } else
    {
      closedLoopControllerThread.stop();
    }
    // Calculate Spark conversion factors
    double positionConversionFactor = config.getGearing().getMechanismToRotorRatio();
    double velocityConversionFactor = config.getGearing().getMechanismToRotorRatio() / 60.0;

    // Set base config options
    sparkBaseConfig.openLoopRampRate(config.getOpenLoopRampRate());
    sparkBaseConfig.closedLoopRampRate(config.getClosedLoopRampRate());
    sparkBaseConfig.inverted(config.getMotorInverted());
    sparkBaseConfig.encoder.positionConversionFactor(positionConversionFactor);
    sparkBaseConfig.encoder.velocityConversionFactor(velocityConversionFactor);
//    sparkBaseConfig.encoder.inverted(config.getEncoderInverted());
    if (config.getEncoderInverted())
    {
      throw new IllegalArgumentException("[ERROR] Spark relative encoder cannot be inverted!");
    }
    // Throw warning about supply stator limits on Spark's
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      DriverStation.reportError("[WARNING] Supply stall currently not supported on Spark", true);
    }
    // Handle stator current limit.
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      sparkBaseConfig.smartCurrentLimit(config.getStatorStallCurrentLimit().getAsInt());
    }
    // Handle voltage compensation.
    if (config.getVoltageCompensation().isPresent())
    {
      sparkBaseConfig.voltageCompensation(config.getVoltageCompensation().get().in(Volts));
    }
    // Setup idle mode.
    if (config.getIdleMode().isPresent())
    {
      sparkBaseConfig.idleMode(config.getIdleMode().get() == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    }
    // Setup starting position
    if (config.getStartingPosition().isPresent())
    {
      sparkRelativeEncoder.setPosition(config.getStartingPosition().get().in(Rotations));
    }
    // Setup external encoder.
    if (config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder instanceof SparkAbsoluteEncoder)
      {
        sparkAbsoluteEncoder = Optional.of((SparkAbsoluteEncoder) externalEncoder);
        sparkBaseConfig.absoluteEncoder.positionConversionFactor(positionConversionFactor);
        sparkBaseConfig.absoluteEncoder.velocityConversionFactor(velocityConversionFactor);
        sparkBaseConfig.absoluteEncoder.inverted(config.getEncoderInverted());

        if (RobotBase.isSimulation())
        {
          if (spark instanceof SparkMax)
          {
            sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkMax) spark));
          } else if (spark instanceof SparkFlex)
          {
            sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkFlex) spark));
          }
          if (config.getStartingPosition().isPresent())
          {
            sparkAbsoluteEncoderSim.ifPresent(enc -> enc.setPosition(config.getStartingPosition().get().in(Rotations)));
          }
        }
      } else
      {
        throw new IllegalArgumentException(
            "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName());
      }

      // Set starting position if external encoder is empty.
      if (config.getStartingPosition().isEmpty())
      {
        sparkRelativeEncoder.setPosition(sparkAbsoluteEncoder.get().getPosition());
      }
    }

    // Configure follower motors
    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        if (follower.getFirst() instanceof SparkMax)
        {
          ((SparkMax) follower.getFirst()).configure(new SparkMaxConfig().follow(spark, follower.getSecond()),
                                                     ResetMode.kNoResetSafeParameters,
                                                     PersistMode.kPersistParameters);

        } else if (follower.getFirst() instanceof SparkFlex)
        {
          ((SparkFlex) follower.getFirst()).configure(new SparkFlexConfig().follow(spark, follower.getSecond()),
                                                      ResetMode.kNoResetSafeParameters,
                                                      PersistMode.kPersistParameters);

        } else
        {
          throw new IllegalArgumentException(
              "Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
      config.clearFollowers();
    }

    return configureSpark(() -> spark.configure(sparkBaseConfig,
                                                ResetMode.kNoResetSafeParameters,
                                                PersistMode.kPersistParameters));
  }

  @Override
  public double getDutyCycle()
  {
    return sparkSim.map(SparkSim::getAppliedOutput).orElseGet(spark::getAppliedOutput);
  }

  @Override
  public SysIdRoutine sysId(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration)
  {
    SysIdRoutine sysIdRoutine = null;
    if (config.getTelemetryName().isEmpty())
    {
      throw new IllegalArgumentException("[ERROR] Missing SmartMotorController telemetry name");
    }
    if (config.getMechanismCircumference().isPresent())
    {
      sysIdRoutine = new SysIdRoutine(new Config(stepVoltage, maxVoltage, testDuration),
                                      new SysIdRoutine.Mechanism(
                                          this::setVoltage,
                                          log -> {
                                            log.motor(config.getTelemetryName().get())
                                               .voltage(
                                                   getVoltage())
                                               .linearVelocity(getMeasurementVelocity())
                                               .linearPosition(getMeasurementPosition());
                                          },
                                          config.getSubsystem()));
    } else
    {
      sysIdRoutine = new SysIdRoutine(new Config(stepVoltage, maxVoltage, testDuration),
                                      new SysIdRoutine.Mechanism(
                                          this::setVoltage,
                                          log -> {
                                            log.motor(config.getTelemetryName().get())
                                               .voltage(
                                                   getVoltage())
                                               .angularPosition(getMechanismPosition())
                                               .angularVelocity(getMechanismVelocity());
                                          },
                                          config.getSubsystem()));
    }
    return sysIdRoutine;
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    spark.set(dutyCycle);
    sparkSim.ifPresent(sparkSim1 -> sparkSim1.setAppliedOutput(dutyCycle));
  }

  @Override
  public Current getSupplyCurrent()
  {
    DriverStation.reportError("[WARNING] Supply currently not supported on Spark", true);
    return null;
  }

  @Override
  public Current getStatorCurrent()
  {
    return sparkSim.map(sim -> Amps.of(sim.getMotorCurrent())).orElseGet(() -> Amps.of(spark.getOutputCurrent()));
  }

  @Override
  public Voltage getVoltage()
  {
    return Volts.of(spark.getAppliedOutput() * spark.getBusVoltage());
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    spark.setVoltage(voltage);
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
    if (sparkAbsoluteEncoder.isPresent() && config.getUseExternalFeedback())
    {
      return RotationsPerSecond.of(sparkAbsoluteEncoder.get().getVelocity());
    }
    return RotationsPerSecond.of(sparkRelativeEncoder.getVelocity());
  }

  @Override
  public Angle getMechanismPosition()
  {
    Angle pos = Rotations.of(sparkRelativeEncoder.getPosition());
    if (sparkAbsoluteEncoder.isPresent() && config.getUseExternalFeedback())
    {
      pos = Rotations.of(sparkAbsoluteEncoder.get().getPosition());
    }
    if (config.getZeroOffset().isPresent())
    {
      pos = pos.minus(config.getZeroOffset().get());
    }
    return pos;
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return RotationsPerSecond.of(getMechanismPosition().in(Rotations) * config.getGearing().getMechanismToRotorRatio());
  }

  @Override
  public Angle getRotorPosition()
  {
    return Rotations.of(getMechanismPosition().in(Rotations) * config.getGearing().getRotorToMechanismRatio());
  }


  /**
   * Iterate the closed loop controller. Feedforward are only applied with profiled pid controllers.
   */
  public void iterateClosedLoopController()
  {
    double pidOutputVoltage = 0;
    double feedforward      = 0.0;
    telemetry.setpointPosition = 0;
    telemetry.setpointVelocity = 0;
    telemetry.velocityControl = false;
    telemetry.motionProfile = false;
    telemetry.statorCurrent = spark.getOutputCurrent();
    telemetry.mechanismPosition = getMechanismPosition();
    telemetry.mechanismVelocity = getMechanismVelocity();
    telemetry.rotorPosition = getRotorPosition();
    telemetry.rotorVelocity = getRotorVelocity();

    if (config.getFeedbackSynchronizationThreshold().isPresent())
    {
      if (sparkAbsoluteEncoder.isPresent())
      {
        if (!Rotations.of(sparkRelativeEncoder.getPosition()).isNear(Rotations.of(sparkAbsoluteEncoder.get()
                                                                                                      .getPosition()),
                                                                     config.getFeedbackSynchronizationThreshold()
                                                                           .get()))
        {
          seedRelativeEncoder();
        }
      }
    }

    if (pidController.isPresent() && setpointPosition.isPresent())
    {
      telemetry.motionProfile = true;
      telemetry.armFeedforward = false;
      telemetry.elevatorFeedforward = false;
      telemetry.simpleFeedforward = false;
      if (config.getArmFeedforward().isPresent())
      {
        telemetry.armFeedforward = true;
        pidOutputVoltage = pidController.get().calculate(getMechanismPosition().in(Rotations),
                                                         setpointPosition.get().in(Rotations));
        feedforward = config.getArmFeedforward().get().calculateWithVelocities(getMechanismPosition().in(Rotations),
                                                                               getMechanismVelocity().in(
                                                                                   RotationsPerSecond),
                                                                               pidController.get()
                                                                                            .getSetpoint().velocity);
      } else if (config.getElevatorFeedforward().isPresent())
      {
        telemetry.elevatorFeedforward = true;
        telemetry.distance = getMeasurementPosition();
        telemetry.linearVelocity = getMeasurementVelocity();
        pidOutputVoltage = pidController.get().calculate(getMeasurementPosition().in(Meters),
                                                         config.convertFromMechanism(setpointPosition.get())
                                                               .in(Meters));
        feedforward = config.getElevatorFeedforward().get().calculateWithVelocities(getMeasurementVelocity().in(
            MetersPerSecond), pidController.get().getSetpoint().velocity);

      } else if (config.getSimpleFeedforward().isPresent())
      {
        telemetry.simpleFeedforward = true;
        feedforward = config.getSimpleFeedforward().get().calculateWithVelocities(getMechanismVelocity().in(
            RotationsPerSecond), pidController.get().getSetpoint().velocity);

      }
      telemetry.setpointPosition = pidController.get().getSetpoint().position;
      telemetry.setpointVelocity = pidController.get().getSetpoint().velocity;

    } else if (simplePidController.isPresent())
    {
      if (setpointPosition.isPresent())
      {
        telemetry.setpointPosition = setpointPosition.get().in(Rotations);
        pidOutputVoltage = simplePidController.get().calculate(setpointPosition.get().in(Rotations),
                                                               setpointPosition.get().in(Rotations));
      } else if (setpointVelocity.isPresent())
      {
        telemetry.velocityControl = true;
        telemetry.setpointVelocity = setpointVelocity.get().in(RotationsPerSecond);
        pidOutputVoltage = simplePidController.get().calculate(setpointVelocity.get().in(RotationsPerSecond));
      }
    }
    if (config.getMechanismUpperLimit().isPresent())
    {
      telemetry.mechanismUpperLimit = getMechanismPosition().gt(config.getMechanismUpperLimit().get());
      if (telemetry.mechanismUpperLimit)
      {
        pidOutputVoltage = feedforward = 0;
      }
    }
    if (config.getMechanismLowerLimit().isPresent())
    {
      telemetry.mechanismLowerLimit = getMechanismPosition().lt(config.getMechanismLowerLimit().get());
      if (telemetry.mechanismLowerLimit)
      {
        pidOutputVoltage = feedforward = 0;
      }
    }
    if (config.getTemperatureCutoff().isPresent())
    {
      telemetry.temperature = getTemperature();
      telemetry.temperatureLimit = telemetry.temperature.gte(config.getTemperatureCutoff().get());
      if (telemetry.temperatureLimit)
      {
        pidOutputVoltage = feedforward = 0;
      }
    }
    telemetry.pidOutputVoltage = pidOutputVoltage;
    telemetry.feedforwardVoltage = feedforward;
    telemetry.outputVoltage = pidOutputVoltage + feedforward;
    if (config.getClosedLoopControllerMaximumVoltage().isPresent())
    {
      double maximumVoltage = config.getClosedLoopControllerMaximumVoltage().get().in(Volts);
      telemetry.outputVoltage = MathUtil.clamp(telemetry.outputVoltage, -maximumVoltage, maximumVoltage);
    }
    setVoltage(Volts.of(telemetry.outputVoltage));
  }

  @Override
  public void updateTelemetry(NetworkTable table)
  {
    if (parentTable.isEmpty())
    {
      parentTable = Optional.of(table);
      if (config.getTelemetryName().isPresent())
      {
        telemetryTable = Optional.of(table.getSubTable(config.getTelemetryName().get()));
        updateTelemetry();
      }
    }
  }

  @Override
  public Temperature getTemperature()
  {
    return Celsius.of(spark.getMotorTemperature());
  }

  @Override
  public void updateTelemetry()
  {
    if (telemetryTable.isPresent() && config.getVerbosity().isPresent())
    {
//      telemetry.temperature = getTemperature();
//      telemetry.mechanismLowerLimit = false;
//      telemetry.mechanismUpperLimit = false;
//      telemetry.temperatureLimit = false;
      telemetry.statorCurrent = spark.getOutputCurrent();
      telemetry.mechanismPosition = getMechanismPosition();
      telemetry.mechanismVelocity = getMechanismVelocity();
      telemetry.rotorPosition = getRotorPosition();
      telemetry.rotorVelocity = getRotorVelocity();
//      config.getMechanismLowerLimit().ifPresent(limit ->
//                                                    telemetry.mechanismLowerLimit = getMechanismPosition().lte(limit));
//      config.getMechanismUpperLimit().ifPresent(limit ->
//                                                    telemetry.mechanismUpperLimit = getMechanismPosition().gte(limit));
//      config.getTemperatureCutoff().ifPresent(limit ->
//                                                  telemetry.temperatureLimit = getTemperature().gte(limit));
      telemetry.publish(telemetryTable.get(), config.getVerbosity().get());
    }
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
