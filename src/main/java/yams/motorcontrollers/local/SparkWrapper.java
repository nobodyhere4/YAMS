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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Supplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerTelemetry;

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
  private       SparkSim                          sparkSim;
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
  private       Optional<AbsoluteEncoder>         sparkAbsoluteEncoder;
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
  private       Optional<Angle>                   setpointPosition;
  /**
   * Setpoint velocity.
   */
  private       Optional<AngularVelocity>         setpointVelocity;
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
  private Optional<NetworkTable> parentTable    = Optional.empty();
  /**
   * {@link SmartMotorController} telemetry table.
   */
  private Optional<NetworkTable> telemetryTable = Optional.empty();


  public SparkWrapper(SparkBase controller, DCMotor motor)
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
      sparkSim = new SparkSim(spark, motor);
      if (spark instanceof SparkMax)
      {
        sparkRelativeEncoderSim = Optional.of(new SparkRelativeEncoderSim((SparkMax) spark));
      } else if (spark instanceof SparkFlex)
      {
        sparkRelativeEncoderSim = Optional.of(new SparkRelativeEncoderSim((SparkFlex) spark));
      }
    }
  }


  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {
    if (RobotBase.isSimulation())
    {
      sparkSim.iterate(mechanismVelocity.in(RotationsPerSecond),
                       RoboRioSim.getVInVoltage(),
                       config.getClosedLoopControlPeriod().in(Second));
      sparkRelativeEncoderSim.ifPresent(sim -> sim.iterate(mechanismVelocity.in(RotationsPerSecond),
                                                           config.getClosedLoopControlPeriod().in(Seconds)));
      sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.iterate(mechanismVelocity.in(
          RotationsPerSecond), config.getClosedLoopControlPeriod().in(Seconds)));
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
    if (pidController.isEmpty())
    {
      simplePidController = config.getSimpleClosedLoopController();
      if (simplePidController.isEmpty())
      {
        throw new IllegalArgumentException("closed-loop controller must not be empty");
      }

      if (config.getWrappingMin().isPresent() && config.getWrappingMax().isPresent())
      {
        if (config.getMechanismCircumference().isPresent())
        {
          throw new IllegalArgumentException("Mechanism circumference must be empty when wrapping is enabled.");
        }
        simplePidController.ifPresent(controller -> controller.enableContinuousInput(config.getWrappingMin().get()
                                                                                           .in(Rotations),
                                                                                     config.getWrappingMax().get()
                                                                                           .in(Rotations)));
      }
      if (config.getClosedLoopTolerance().isPresent())
      {
        if (config.getMechanismCircumference().isPresent())
        {
          simplePidController.ifPresent(controller -> controller.setTolerance(config.convertFromMechanism(config.getClosedLoopTolerance()
                                                                                                                .get())
                                                                                    .in(Meters)));
        } else
        {
          simplePidController.ifPresent(controller -> controller.setTolerance(config.getClosedLoopTolerance().get()
                                                                                    .in(Rotations)));
        }

      }
    } else
    {
      if (config.getWrappingMin().isPresent() && config.getWrappingMax().isPresent())
      {
        if (config.getMechanismCircumference().isPresent())
        {
          throw new IllegalArgumentException("Mechanism circumference must be empty when wrapping is enabled.");
        }
        pidController.ifPresent(controller -> controller.enableContinuousInput(config.getWrappingMin().get()
                                                                                     .in(Rotations),
                                                                               config.getWrappingMax().get()
                                                                                     .in(Rotations)));
      }
      if (config.getClosedLoopTolerance().isPresent())
      {
        if (config.getMechanismCircumference().isPresent())
        {
          pidController.ifPresent(controller -> controller.setTolerance(config.convertFromMechanism(config.getClosedLoopTolerance()
                                                                                                          .get())
                                                                              .in(Meters)));
        } else
        {
          pidController.ifPresent(controller -> controller.setTolerance(config.getClosedLoopTolerance().get()
                                                                              .in(Rotations)));
        }
      }

    }
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
    closedLoopControllerThread.startPeriodic(config.getClosedLoopControlPeriod().in(Second));

    double positionConversionFactor = config.getGearing().getRotorToMechanismRatio();
    double velocityConversionFactor = config.getGearing().getRotorToMechanismRatio() / 60.0;

    sparkBaseConfig.openLoopRampRate(config.getOpenLoopRampRate());
    sparkBaseConfig.closedLoopRampRate(config.getClosedLoopRampRate());
    sparkBaseConfig.inverted(config.getMotorInverted());
    sparkBaseConfig.encoder.positionConversionFactor(positionConversionFactor);
    sparkBaseConfig.encoder.velocityConversionFactor(velocityConversionFactor);
    sparkBaseConfig.encoder.inverted(config.getEncoderInverted());
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      DriverStation.reportError("[WARNING] Supply stall currently not supported on Spark", true);
    }
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      sparkBaseConfig.smartCurrentLimit(config.getStatorStallCurrentLimit().getAsInt());
    }
    if (config.getVoltageCompensation().isPresent())
    {
      sparkBaseConfig.voltageCompensation(config.getVoltageCompensation().getAsDouble());
    }
    if (config.getIdleMode().isPresent())
    {
      sparkBaseConfig.idleMode(config.getIdleMode().get() == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    }
    if (config.getStartingPosition().isPresent())
    {
      sparkRelativeEncoder.setPosition(config.getStartingPosition().get().in(Rotations));
    }
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
        if (config.getStartingPosition().isEmpty())
        {
          sparkRelativeEncoder.setPosition(sparkAbsoluteEncoder.get().getPosition());
        }

      }
      throw new IllegalArgumentException(
          "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName());
    }

    return configureSpark(() -> spark.configure(sparkBaseConfig,
                                                ResetMode.kNoResetSafeParameters,
                                                PersistMode.kPersistParameters));
  }

  @Override
  public double getDutyCycle()
  {
    return spark.get();
  }

  @Override
  public Command sysId(VoltageUnit maxVoltage, VelocityUnit<VoltageUnit> stepVoltage, TimeUnit testDuration)
  {
    return null;
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    spark.set(dutyCycle);
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
    return Amps.of(spark.getOutputCurrent());
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
      telemetry.temperature = Celsius.of(spark.getMotorTemperature());
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
  public void updateTelemetry()
  {
    if (telemetryTable.isPresent() && config.getVerbosity().isPresent())
    {
      telemetry.publish(telemetryTable.get(), config.getVerbosity().get());
    }
  }
}
