package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class Elevator extends SmartPositionalMechanism
{

  /**
   * Simulation for the elevator
   */
  private Optional<ElevatorSim> m_sim = Optional.empty();
  /**
   * Config class for the elevator.
   */
  private ElevatorConfig        m_config;

  /**
   * Construct the {@link Elevator} class for easy manipulation of an elevator.
   *
   * @param config {@link ElevatorConfig} to set.
   */
  public Elevator(ElevatorConfig config)
  {
    m_config = config;
    m_motor = config.getMotor();
    m_subsystem = config.getMotor().getConfig().getSubsystem();
    if (config.getTelemetryName().isPresent())
    {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard")
                                               .getSubTable(config.getTelemetryName().get());
      m_telemetry.setupTelemetry(table);
      m_telemetry.units.set("Meters");
      m_motor.updateTelemetry(table);
    }
    config.applyConfig();

    if (RobotBase.isSimulation())
    {
      SmartMotorController motor = config.getMotor();
      if (config.getCarriageMass().isEmpty())
      {
        throw new IllegalArgumentException("Carriage mass cannot be empty");
      }
      if (config.getMinimumHeight().isEmpty())
      {
        throw new IllegalArgumentException("Minimum height cannot be empty");
      }
      if (config.getMaximumHeight().isEmpty())
      {
        throw new IllegalArgumentException("Maximum height cannot be empty");
      }
      if (config.getStartingHeight().isEmpty())
      {
        throw new IllegalArgumentException("Starting height cannot be empty");
      }
      m_sim = Optional.of(new ElevatorSim(motor.getDCMotor(),
                                          motor.getConfig().getGearing().getMechanismToRotorRatio(),
                                          config.getCarriageMass().get().in(Kilograms),
                                          config.getDrumRadius().in(Meters),
                                          config.getMinimumHeight().get().in(Meters),
                                          config.getMaximumHeight().get().in(Meters),
                                          true,
                                          config.getStartingHeight().get().in(Meters),
                                          0.01 / 4096, 0.01 / 4096));

      mechanismWindow = new Mechanism2d(config.getMaximumHeight().get().in(Meters) * 2,
                                        config.getMaximumHeight().get().in(Meters) * 2);

      if (m_motor.getConfig().getMechanismLowerLimit().isPresent())
      {
        mechanismWindow.getRoot(
                           "MinSoft",
                           config.getMaximumHeight().get().minus(Inches.of(6)).in(Meters), 0)
                       .append(new MechanismLigament2d(
                           "Limit",
                           m_motor.getConfig().convertFromMechanism(m_motor.getConfig().getMechanismLowerLimit().get())
                                  .in(Meters),
                           config.getAngle().in(Degrees),
                           3,
                           new Color8Bit(Color.kYellow)
                       ));
      }
      if (m_motor.getConfig().getMechanismUpperLimit().isPresent())
      {
        mechanismWindow.getRoot(
                           "MaxSoft",
                           config.getMaximumHeight().get().plus(Inches.of(6)).in(Meters), 0)
                       .append(new MechanismLigament2d(
                           "Limit",
                           m_motor.getConfig().convertFromMechanism(m_motor.getConfig().getMechanismUpperLimit().get())
                                  .in(Meters),
                           config.getAngle().in(Degrees),
                           3,
                           new Color8Bit(Color.kHotPink)
                       ));
      }
      mechanismWindow.getRoot(
                         "MinHard",
                         config.getMaximumHeight().get().minus(Inches.of(8)).in(Meters), 0)
                     .append(new MechanismLigament2d(
                         "Limit",
                         config.getMinimumHeight().get().in(Meters),
                         config.getAngle().in(Degrees),
                         3,
                         new Color8Bit(Color.kRed)
                     ));
      mechanismWindow.getRoot(
                         "MaxHard",
                         config.getMaximumHeight().get().plus(Inches.of(8)).in(Meters), 0)
                     .append(new MechanismLigament2d(
                         "Limit",
                         config.getMaximumHeight().get().in(Meters),
                         config.getAngle().in(Degrees),
                         3,
                         new Color8Bit(Color.kLimeGreen)
                     ));
      mechanismRoot = mechanismWindow.getRoot(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() + "Root" : "ElevatorRoot",
          config.getMaximumHeight().get().in(Meters), 0);
      mechanismLigament = mechanismRoot.append(new MechanismLigament2d(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() : "Elevator",
          config.getStartingHeight().get().in(Meters),
          config.getAngle().in(Degrees), 6, config.getSimColor()));
      SmartDashboard.putData(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() + "/mechanism" : "Elevator/mechanism",
          mechanismWindow);
    }
  }

  /**
   * Update the mechanism's telemetry.
   */
  public void updateTelemetry()
  {
    m_telemetry.positionPublisher.set(m_motor.getMeasurementPosition().in(Meters));
    m_motor.getMechanismSetpoint().ifPresent(m_setpoint -> m_telemetry.setpointPublisher.set(m_motor.getConfig()
                                                                                                    .convertFromMechanism(
                                                                                                        m_setpoint)
                                                                                                    .in(Meters)));
    m_motor.updateTelemetry();
  }

  /**
   * Iterate sim
   */
  public void simIterate()
  {
    if (m_sim.isPresent())
    {
      m_sim.get().setInput(m_motor.getDutyCycle() * RoboRioSim.getVInVoltage());
      m_sim.get().update(m_motor.getConfig().getClosedLoopControlPeriod().in(Seconds));

      m_motor.simIterate(m_motor.getConfig()
                                .convertToMechanism(MetersPerSecond.of(m_sim.get().getVelocityMetersPerSecond())));
      // It is impossible for an elevator to go bellow the minimum height, it would break...
      if (m_config.getMinimumHeight().isPresent() && getHeight().lt(m_config.getMinimumHeight().get()))
      {
        m_motor.simIterate(RotationsPerSecond.of(0));
        m_motor.setEncoderPosition(m_config.getMinimumHeight().get());
      } else
      {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.get().getCurrentDrawAmps()));
      }
      mechanismLigament.setLength(getHeight().in(Meters));
    }
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that  sets the elevator height, stops immediately.
   */
  public Command setHeight(Distance height)
  {
    return Commands.run(() -> m_motor.setPosition(height), m_subsystem);
  }

  /**
   * Set the dutycycle of the elevator motor.
   *
   * @param dutycycle [-1,1] to set.
   * @return {@link Command}
   */
  public Command set(double dutycycle)
  {
    return Commands.run(() -> m_motor.setDutyCycle(dutycycle), m_subsystem);
  }

  /**
   * Set the voltage of the elevator.
   *
   * @param volts {@link Voltage} of the Elevator to set.
   * @return {@link Command}
   */
  public Command setVoltage(Voltage volts)
  {
    return Commands.run(() -> m_motor.setVoltage(volts), m_subsystem);
  }

  /**
   * Get the Height of the Elevator.
   *
   * @return {@link Distance} of the Elevator.
   */
  public Distance getHeight()
  {
    return m_motor.getMeasurementPosition();
  }

  /**
   * Get the linear velocity of the Elevator.
   *
   * @return {@link LinearVelocity} of the elevator.
   */
  public LinearVelocity getVelocity()
  {
    return m_motor.getMeasurementVelocity();
  }


  /**
   * Elevator is near a height.
   *
   * @param height {@link Distance} to be near.
   * @param within {@link Distance} within.
   * @return Trigger on when the elevator is near another height.
   */
  public Trigger isNear(Distance height, Distance within)
  {
    return new Trigger(() -> getHeight().isNear(height, within));
  }

  /**
   * Elevator is at max, defined by the soft limit or hard limit on the elevator.
   *
   * @return Maximum height for the elevator.
   */
  public Trigger max()
  {
    if (m_motor.getConfig().getMechanismUpperLimit().isPresent())
    {
      return new Trigger(gte(m_motor.getConfig()
                                    .convertFromMechanism(m_motor.getConfig().getMechanismUpperLimit().get())));
    }
    if (m_config.getMaximumHeight().isEmpty())
    {
      return gte(m_config.getMaximumHeight().get());
    }
    throw new IllegalArgumentException("Upper soft and hard limit is empty. Cannot create maximum trigger.");
  }

  /**
   * Minimum height of the elevator given by the soft limit or hard limit of the elevator.
   *
   * @return {@link Trigger} on minimum of the elevator.
   */
  public Trigger min()
  {
    if (m_motor.getConfig().getMechanismLowerLimit().isPresent())
    {
      return new Trigger(gte(m_motor.getConfig()
                                    .convertFromMechanism(m_motor.getConfig().getMechanismLowerLimit().get())));
    }
    if (m_config.getMinimumHeight().isEmpty())
    {
      return gte(m_config.getMinimumHeight().get());
    }
    throw new IllegalArgumentException("Lower soft and hard limit is empty. Cannot create maximum trigger.");
  }


  /**
   * Between two heights.
   *
   * @param start Start height.
   * @param end   End height.
   * @return {@link Trigger}
   */
  public Trigger between(Distance start, Distance end)
  {
    return gte(start).and(lte(end));
  }

  /**
   * Less than or equal to height
   *
   * @param height {@link Distance} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Distance height)
  {
    return new Trigger(() -> getHeight().lte(height));
  }

  /**
   * Greater than or equal to height.
   *
   * @param height Height to check against.
   * @return {@link Trigger} for elevator.
   */
  public Trigger gte(Distance height)
  {
    return new Trigger(() -> getHeight().gte(height));
  }

  /**
   * Create the SysId routine and commands to run the SysId tests. The SysId test will run the mechanism up then down at
   * a constant speed, then run the mechanism up at an increasing speed and down at an increasing speed. Requires the
   * maximum and minimum limit to be set. Runs the mechanism within 1 degree of the maximum and minimum.
   *
   * @param maximumVoltage Maximum {@link Voltage} to give to the elevator, is the voltage given to run the elevator up
   *                       at a static speed.
   * @param step           Step {@link Voltage} to give to the elevator.
   * @param duration       SysId test duration.
   * @return {@link edu.wpi.first.wpilibj2.command.SequentialCommandGroup} running the SysId commands.
   */
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    SysIdRoutine               routine     = m_motor.sysId(maximumVoltage, step, duration);
    SmartMotorControllerConfig motorConfig = m_motor.getConfig();
    Distance                   max;
    Distance                   min;
    if (m_motor.getConfig().getMechanismUpperLimit().isPresent())
    {
      max = motorConfig.convertFromMechanism(m_motor.getConfig().getMechanismUpperLimit().get())
                       .minus(Centimeters.of(1));
    } else if (m_config.getMaximumHeight().isPresent())
    {
      max = m_config.getMaximumHeight().get().minus(Centimeters.of(1));
    } else
    {
      throw new IllegalArgumentException("No upper soft or hard limit is set. Cannot create SysId command.");
    }
    if (m_motor.getConfig().getMechanismLowerLimit().isPresent())
    {
      min = motorConfig.convertFromMechanism(m_motor.getConfig().getMechanismLowerLimit().get())
                       .plus(Centimeters.of(1));
    } else if (m_config.getMinimumHeight().isPresent())
    {
      min = m_config.getMinimumHeight().get().plus(Centimeters.of(1));
    } else
    {
      throw new IllegalArgumentException("No lower soft or hard limit is set. Cannot create SysId command.");
    }
    Trigger maxTrigger = gte(max);
    Trigger minTrigger = lte(min);

    Command group = routine.dynamic(Direction.kForward).until(maxTrigger)
                           .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
                           .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
                           .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger));
    if (m_config.getTelemetryName().isPresent())
    {
      group = group.andThen(Commands.print(m_config.getTelemetryName().get() + " SysId test done."));
    }
    return group;
  }
}
