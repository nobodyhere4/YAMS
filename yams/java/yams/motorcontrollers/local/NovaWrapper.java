package yams.motorcontrollers.local;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.List;
import java.util.Optional;
import yams.motorcontrollers.SimSupplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * Nova wrapper for {@link SmartMotorController}
 */
public class NovaWrapper extends SmartMotorController
{

  /**
   * Thrifty Nova controller.
   */
  private final ThriftyNova          m_nova;
  /**
   * Motor characteristics controlled by the {@link ThriftyNova}.
   */
  private final DCMotor              m_motor;
  /**
   * Sim for ThriftyNova's.
   */
  private       Optional<DCMotorSim> m_sim = Optional.empty();

  /**
   * Construct the Nova Wrapper for the generic {@link SmartMotorController}.
   *
   * @param controller {@link ThriftyNova} to use.
   * @param motor      {@link DCMotor} connected to the {@link ThriftyNova}.
   * @param config     {@link SmartMotorControllerConfig} to apply to the {@link ThriftyNova}.
   */
  public NovaWrapper(ThriftyNova controller, DCMotor motor, SmartMotorControllerConfig config)
  {
    m_nova = controller;
    this.m_motor = motor;
    this.config = config;
    setupSimulation();
    applyConfig(config);
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      var setupRan = m_sim.isPresent();
      if (!setupRan)
      {
        m_sim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(m_motor,
                                                                              0.001,
                                                                              config.getGearing()
                                                                                    .getRotorToMechanismRatio()),
                                           m_motor,
                                           1.0 / 1024.0, 0));

        setSimSupplier(new SimSupplier()
        {
          boolean simUpdated = false;
          boolean inputFed   = false;

          @Override
          public void updateSimState()
          {
            if (!inputFed)
            {
              m_sim.get().setInput(getDutyCycle() * RoboRioSim.getVInVoltage());
            }
            if (!simUpdated)
            {
              starveInput();
              m_sim.get().update(getConfig().getClosedLoopControlPeriod().in(Seconds));
              feedUpdateSim();
            }
          }

          @Override
          public boolean getUpdatedSim()
          {
            return simUpdated;
          }

          @Override
          public void feedUpdateSim()
          {
            simUpdated = true;
          }

          @Override
          public void starveUpdateSim()
          {
            simUpdated = false;
          }

          @Override
          public boolean isInputFed()
          {
            return simUpdated;
          }

          @Override
          public void feedInput()
          {
            inputFed = true;
          }

          @Override
          public void starveInput()
          {
            inputFed = false;
          }

          @Override
          public void setMechanismStatorDutyCycle(double dutyCycle)
          {
            inputFed = true;
            m_sim.get().setInputVoltage(dutyCycle * getMechanismSupplyVoltage().in(Volts));
          }

          @Override
          public Voltage getMechanismSupplyVoltage()
          {
            return Volts.of(RoboRioSim.getVInVoltage());
          }

          @Override
          public Voltage getMechanismStatorVoltage()
          {
            return Volts.of(m_motor.getVoltage(m_sim.get().getTorqueNewtonMeters(),
                                               m_sim.get().getAngularVelocityRadPerSec()));
          }

          @Override
          public void setMechanismStatorVoltage(Voltage volts)
          {
            inputFed = true;
            m_sim.get().setInputVoltage(volts.in(Volts));
          }

          @Override
          public Angle getMechanismPosition()
          {
            return m_sim.get().getAngularPosition();
          }

          @Override
          public void setMechanismPosition(Angle position)
          {
            m_sim.get().setAngle(position.in(Radians));
          }

          @Override
          public Angle getRotorPosition()
          {
            return getMechanismPosition().times(config.getGearing().getMechanismToRotorRatio());
          }

          @Override
          public AngularVelocity getMechanismVelocity()
          {
            return m_sim.get().getAngularVelocity();
          }

          @Override
          public void setMechanismVelocity(AngularVelocity velocity)
          {
            m_sim.get().setAngularVelocity(velocity.in(RadiansPerSecond));
          }

          @Override
          public AngularVelocity getRotorVelocity()
          {
            return getMechanismVelocity().times(config.getGearing().getMechanismToRotorRatio());
          }

          @Override
          public Current getCurrentDraw()
          {
            return Amps.of(m_sim.get().getCurrentDrawAmps());
          }
        });
      }

      config.getStartingPosition().ifPresent(mechPos -> {
        m_sim.get().setAngle(mechPos.in(Radians));
      });
    }
  }

  @Override
  public void seedRelativeEncoder()
  {
    DriverStation.reportWarning("[WARNING] NovaWrapper.seedRelativeEncoder() is not supported", false);
  }

  @Override
  public void synchronizeRelativeEncoder()
  {
    if (!RobotBase.isSimulation())
    {
      DriverStation.reportWarning(
          "[WARNING] NovaWrapper.synchronizeRelativeEncoder() is not supported on ThriftyNova's.",
          false);
    }
  }

  @Override
  public void simIterate()
  {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent())
    {
      if (!m_simSupplier.get().getUpdatedSim())
      {
        m_simSupplier.get().updateSimState();
        m_simSupplier.get().starveUpdateSim();
      }
      m_sim.ifPresent(sim -> {
        sim.setAngularVelocity(m_simSupplier.get().getMechanismVelocity().in(RadiansPerSecond));
        sim.update(config.getClosedLoopControlPeriod().in(Seconds));
      });
    }
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
//    m_sim.ifPresent(dcMotorSim -> dcMotorSim.setAngularVelocity(velocity.in(RadiansPerSecond)));
    DriverStation.reportWarning("[WARNING] NovaWrapper.setEncoderVelocity() is not supported on ThriftyNova's.", false);
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    DriverStation.reportWarning("[WARNING] NovaWrapper.setEncoderVelocity() is not supported on ThriftyNova's.", false);
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    m_nova.setEncoderPosition(angle.in(Rotations));
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismPosition(angle));
  }

  @Override
  public void setEncoderPosition(Distance distance)
  {
    setEncoderPosition(config.convertToMechanism(distance));
  }

  @Override
  public void setPosition(Angle angle)
  {
    setpointPosition = Optional.ofNullable(angle);
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
    setpointVelocity = Optional.ofNullable(angle);
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
        throw new IllegalArgumentException("[ERROR] closed loop controller must not be empty");
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

    // Ramp rates
    m_nova.setRampUp(config.getClosedLoopRampRate().in(Seconds));
    m_nova.setRampDown(config.getClosedLoopRampRate().in(Seconds));
    if (config.getOpenLoopRampRate().gt(Seconds.of(0)))
    {
      throw new IllegalArgumentException(
          "[Error] ThriftyNova does not support separate closed loop and open loop ramp rates, using the SmartMotorControllerConfig.withClosedLoopRampRate() as both.");
    }

    // Inversions
    m_nova.setInverted(config.getMotorInverted());
    if (config.getEncoderInverted())
    {
      throw new IllegalArgumentException("[ERROR] ThriftyNova does not support encoder inversions.");
    }

    // Current limits
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      m_nova.setMaxCurrent(CurrentType.SUPPLY, config.getSupplyStallCurrentLimit().getAsInt());
    }
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      m_nova.setMaxCurrent(CurrentType.STATOR, config.getStatorStallCurrentLimit().getAsInt());
    }

    // Voltage Compensation
    if (config.getVoltageCompensation().isPresent())
    {
      m_nova.setVoltageCompensation(config.getVoltageCompensation().get().in(Volts));
    }

    // Setup idle mode.
    if (config.getIdleMode().isPresent())
    {
      m_nova.setBrakeMode(config.getIdleMode().get() == MotorMode.BRAKE);
    }

    // Starting Position
    if (config.getStartingPosition().isPresent())
    {
      setEncoderPosition(config.getStartingPosition().get());
    }

    // External Encoder
    if (config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.QUAD)
      {
        m_nova.useEncoderType(EncoderType.QUAD);
      } else if (externalEncoder == EncoderType.ABS)
      {
        m_nova.useEncoderType(EncoderType.ABS);
      } else
      {
        throw new IllegalArgumentException(
            "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName() +
            ".\n\tPlease use an `EncoderType` instead.");
      }

      if (config.getStartingPosition().isEmpty())
      {
        if (externalEncoder == EncoderType.ABS)
        {
          m_nova.setEncoderPosition(m_nova.getPositionAbs());
        }
      }
    }

    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        if (follower.getFirst() instanceof ThriftyNova)
        {
          ((ThriftyNova) follower.getFirst()).follow(m_nova.getID());
          ((ThriftyNova) follower.getFirst()).setInverted(follower.getSecond());
        } else
        {
          throw new IllegalArgumentException(
              "[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
    }

    if (config.getZeroOffset().isPresent())
    {
      DriverStation.reportError("[ERROR] ThriftyNova does not support zero offset, or we have not implemented this.",
                                true);
    }

    if (config.getDiscontinuityPoint().isPresent())
    {
      throw new IllegalArgumentException(
          "[ERROR] ThriftyNova does not support discontinuity points, or we have not implemented this.");
    }

    return true;
  }

  @Override
  public double getDutyCycle()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getMechanismStatorVoltage().in(Volts) /
                                       m_simSupplier.get().getMechanismSupplyVoltage().in(Volts) : m_nova.get();
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorDutyCycle(dutyCycle));
    m_nova.set(dutyCycle);
  }

  @Override
  public Optional<Current> getSupplyCurrent()
  {
    if (m_simSupplier.isPresent())
    {
      return Optional.of(Amps.of(RoboRioSim.getVInCurrent()));
    }
    return Optional.of(Amps.of(m_nova.getSupplyCurrent()));
  }

  @Override
  public Current getStatorCurrent()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getCurrentDraw() : Amps.of(m_nova.getStatorCurrent());
  }

  @Override
  public Voltage getVoltage()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getMechanismStatorVoltage() : Volts.of(m_nova.getVoltage());
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorVoltage(voltage));
    m_nova.setVoltage(voltage);
  }

  @Override
  public DCMotor getDCMotor()
  {
    return m_motor;
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
    if (m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getMechanismVelocity();
    }
    if (config.getUseExternalFeedback() && config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.ABS)
      {
        // Do nothing since attached absolute encoders do not give their velocity.
        // There should be an alert thrown here; but Alerts are not thread-safe.
      } else if (externalEncoder == EncoderType.QUAD)
      {
        // There should be an alert thrown here; but Alerts are not thread-safe.
        return RotationsPerSecond.of(
            m_nova.getVelocityQuad() * config.getExternalEncoderGearing().getRotorToMechanismRatio());
      }
    }
    return getRotorVelocity().times(config.getGearing().getRotorToMechanismRatio());
  }

  @Override
  public Angle getMechanismPosition()
  {
    if (m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getMechanismPosition();
    }
    if (config.getUseExternalFeedback() && config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.ABS)
      {
        return Rotations.of(m_nova.getPositionAbs() * config.getExternalEncoderGearing().getRotorToMechanismRatio());
      } else if (externalEncoder == EncoderType.QUAD)
      {
        return Rotations.of(m_nova.getPositionQuad() * config.getExternalEncoderGearing().getRotorToMechanismRatio());
      }
    }
    return getRotorPosition().times(config.getGearing().getRotorToMechanismRatio());
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getRotorVelocity();
    }
    return RotationsPerSecond.of(m_nova.getVelocity());
  }

  @Override
  public Angle getRotorPosition()
  {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getRotorPosition();
    }
    return Rotations.of(m_nova.getPosition());
  }

  @Override
  public void setMotorInverted(boolean inverted)
  {
    config.withMotorInverted(inverted);
    m_nova.setInverted(inverted);
  }

  @Override
  public void setEncoderInverted(boolean inverted)
  {
    config.withEncoderInverted(inverted);
    m_nova.setInverted(inverted);
  }

  @Override
  public void setMotionProfileMaxVelocity(LinearVelocity maxVelocity)
  {
    if (pidController.isPresent())
    {
      ProfiledPIDController ctr = pidController.get();
      ctr.setConstraints(new Constraints(maxVelocity.in(MetersPerSecond), ctr.getConstraints().maxAcceleration));
      config.withClosedLoopController(ctr);
      pidController = Optional.of(ctr);
    }
  }

  @Override
  public void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration)
  {
    if (pidController.isPresent())
    {
      ProfiledPIDController ctr = pidController.get();
      ctr.setConstraints(new Constraints(ctr.getConstraints().maxVelocity,
                                         maxAcceleration.in(MetersPerSecondPerSecond)));
      config.withClosedLoopController(ctr);
      pidController = Optional.of(ctr);
    }
  }

  @Override
  public void setMotionProfileMaxVelocity(AngularVelocity maxVelocity)
  {
    if (pidController.isPresent())
    {
      ProfiledPIDController ctr = pidController.get();
      ctr.setConstraints(new Constraints(maxVelocity.in(RotationsPerSecond), ctr.getConstraints().maxAcceleration));
      config.withClosedLoopController(ctr);
      pidController = Optional.of(ctr);
    }
  }

  @Override
  public void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration)
  {
    if (pidController.isPresent())
    {
      ProfiledPIDController ctr = pidController.get();
      ctr.setConstraints(new Constraints(ctr.getConstraints().maxVelocity,
                                         maxAcceleration.in(RotationsPerSecondPerSecond)));
      config.withClosedLoopController(ctr);
      pidController = Optional.of(ctr);
    }
  }

  @Override
  public void setKp(double kP)
  {
    simplePidController.ifPresent(simplePidController -> {
      simplePidController.setP(kP);
      config.withClosedLoopController(simplePidController);
    });
    pidController.ifPresent(pidController -> {
      pidController.setP(kP);
      config.withClosedLoopController(pidController);
    });
  }

  @Override
  public void setKi(double kI)
  {
    simplePidController.ifPresent(simplePidController -> {
      simplePidController.setI(kI);
      config.withClosedLoopController(simplePidController);
    });
    pidController.ifPresent(pidController -> {
      pidController.setI(kI);
      config.withClosedLoopController(pidController);
    });

  }

  @Override
  public void setKd(double kD)
  {
    simplePidController.ifPresent(simplePidController -> {
      simplePidController.setP(kD);
      config.withClosedLoopController(simplePidController);
    });
    pidController.ifPresent(pidController -> {
      pidController.setP(kD);
      config.withClosedLoopController(pidController);
    });
  }

  @Override
  public void setFeedback(double kP, double kI, double kD)
  {
    setKp(kP);
    setKi(kI);
    setKd(kD);
  }

  @Override
  public void setKs(double kS)
  {
    config.getSimpleFeedforward().ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKs(kS);
      config.withFeedforward(simpleMotorFeedforward);
    });
    config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKs(kS);
      config.withFeedforward(armFeedforward);
    });
    config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKs(kS);
      config.withFeedforward(elevatorFeedforward);
    });
  }

  @Override
  public void setKv(double kV)
  {
    config.getSimpleFeedforward().ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKv(kV);
      config.withFeedforward(simpleMotorFeedforward);
    });
    config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKv(kV);
      config.withFeedforward(armFeedforward);
    });
    config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKv(kV);
      config.withFeedforward(elevatorFeedforward);
    });
  }

  @Override
  public void setKa(double kA)
  {
    config.getSimpleFeedforward().ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKa(kA);
      config.withFeedforward(simpleMotorFeedforward);
    });
    config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKs(kA);
      config.withFeedforward(armFeedforward);
    });
    config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKa(kA);
      config.withFeedforward(elevatorFeedforward);
    });
  }

  @Override
  public void setKg(double kG)
  {
    config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKg(kG);
      config.withFeedforward(armFeedforward);
    });
    config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKg(kG);
      config.withFeedforward(elevatorFeedforward);
    });
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA, double kG)
  {
    setKs(kS);
    setKv(kV);
    setKa(kA);
    setKg(kG);
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit)
  {
    config.withStatorCurrentLimit(currentLimit);
    m_nova.setMaxCurrent(CurrentType.STATOR, currentLimit.in(Amps));
  }

  @Override
  public void setSupplyCurrentLimit(Current currentLimit)
  {
    config.withSupplyCurrentLimit(currentLimit);
    m_nova.setMaxCurrent(CurrentType.SUPPLY, currentLimit.in(Amps));
  }

  @Override
  public void setClosedLoopRampRate(Time rampRate)
  {
    config.withClosedLoopRampRate(rampRate);
    m_nova.setRampUp(rampRate.in(Seconds));
    m_nova.setRampDown(rampRate.in(Seconds));
  }

  @Deprecated
  /// Unsupported
  public void setOpenLoopRampRate(Time rampRate)
  {
  }

  @Override
  public void setMeasurementUpperLimit(Distance upperLimit)
  {
    if (config.getMechanismCircumference().isPresent() && config.getMechanismLowerLimit().isPresent())
    {
      config.withSoftLimit(config.convertFromMechanism(config.getMechanismLowerLimit().get()), upperLimit);
    }
  }

  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit)
  {
    if (config.getMechanismCircumference().isPresent() && config.getMechanismUpperLimit().isPresent())
    {
      config.withSoftLimit(lowerLimit, config.convertFromMechanism(config.getMechanismUpperLimit().get()));
    }
  }

  @Override
  public void setMechanismUpperLimit(Angle upperLimit)
  {
    config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      config.withSoftLimit(lowerLimit, upperLimit);
    });
  }

  @Override
  public void setMechanismLowerLimit(Angle lowerLimit)
  {
    config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      config.withSoftLimit(lowerLimit, upperLimit);
    });
  }

  @Override
  public Temperature getTemperature()
  {
    return Celsius.of(m_nova.getTemperature());
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return config;
  }

  @Override
  public Object getMotorController()
  {
    return m_nova;
  }

  @Override
  public Object getMotorControllerConfig()
  {
    DriverStation.reportWarning(
        "[WARNING] Thrifty Nova's have no configuration class, returning the ThriftyNova Object.",
        true);
    return m_nova;
  }

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields()
  {
    return Pair.of(Optional.empty(), Optional.empty());
  }
}
