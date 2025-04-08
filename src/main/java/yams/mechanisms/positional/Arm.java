package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import yams.mechanisms.config.ArmConfig;
import yams.motorcontrollers.SmartMotorController;

public class Arm extends SmartPositionalMechanism
{

  /**
   * Simulation for the arm.
   */
  private Optional<SingleJointedArmSim> m_sim = Optional.empty();
  /**
   * Arm config.
   */
  private ArmConfig                     config;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param config {@link ArmConfig} to use.
   */
  public Arm(ArmConfig config)
  {
    this.config = config;
    m_motor = config.getMotor();
    m_subsystem = config.getMotor().getConfig().getSubsystem();
    // Seed the relative encoder
    if (m_motor.getConfig().getExternalEncoder().isPresent())
    {
      m_motor.seedRelativeEncoder();
    }
    if (config.getTelemetryName().isPresent())
    {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard")
                                               .getSubTable(config.getTelemetryName().get());
      m_telemetry.setupTelemetry(table);
      m_telemetry.units.set("Degrees");
      m_motor.updateTelemetry(table);
    }
    config.applyConfig();

    if (RobotBase.isSimulation())
    {
      SmartMotorController motor = config.getMotor();
      if (config.getLength().isEmpty())
      {
        throw new IllegalArgumentException("Arm length is empty. Cannot create simulation.");
      }
      if (config.getLowerHardLimit().isEmpty())
      {
        throw new IllegalArgumentException("Arm lower hard limit is empty. Cannot create simulation.");
      }
      if (config.getUpperHardLimit().isEmpty())
      {
        throw new IllegalArgumentException("Arm upper hard limit is empty. Cannot create simulation.");
      }
      if (config.getStartingAngle().isEmpty())
      {
        throw new IllegalArgumentException("Arm starting angle is empty. Cannot create simulation.");
      }
      m_sim = Optional.of(new SingleJointedArmSim(motor.getDCMotor(),
                                                  motor.getConfig().getGearing().getRotorToMechanismRatio(),
                                                  config.getMOI(),
                                                  config.getLength().get().in(Meters),
                                                  config.getLowerHardLimit().get().in(Radians),
                                                  config.getUpperHardLimit().get().in(Radians),
                                                  true,
                                                  config.getStartingAngle().get().in(Radians),
                                                  0.02 / 4096.0,
                                                  0.0));// Add noise with a std-dev of 1 tick

      mechanismWindow = new Mechanism2d(config.getLength().get().in(Meters) * 2,
                                        config.getLength().get().in(Meters) * 2);
      mechanismRoot = mechanismWindow.getRoot(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() + "Root" : "ArmRoot",
          config.getLength().get().in(Meters), config.getLength().get().in(Meters));
      mechanismLigament = mechanismRoot.append(new MechanismLigament2d(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() : "Arm",
          config.getLength().get().in(Meters),
          config.getStartingAngle().get().in(Degrees), 6, config.getSimColor()));
      MechanismLigament2d maxLigament = mechanismRoot.append(new MechanismLigament2d("Max",
                                                                                     Inch.of(3).in(Meters),
                                                                                     config.getUpperHardLimit().get()
                                                                                           .in(Degrees),
                                                                                     4,
                                                                                     new Color8Bit(Color.kLimeGreen)));
      MechanismLigament2d minLigament = mechanismRoot.append(new MechanismLigament2d("Min", Inch.of(3).in(Meters),
                                                                                     config.getLowerHardLimit().get()
                                                                                           .in(Degrees),
                                                                                     4, new Color8Bit(Color.kRed)));
      SmartDashboard.putData(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() + "/mechanism" : "Arm/mechanism",
                             mechanismWindow);
    }
  }

  /**
   * Update the mechanism's telemetry.
   */
  public void updateTelemetry()
  {
    m_telemetry.positionPublisher.set(m_motor.getMechanismPosition().in(Degrees));
    m_motor.getMechanismSetpoint().ifPresent(m_setpoint -> m_telemetry.setpointPublisher.set(m_setpoint.in(Degrees)));
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

      m_motor.simIterate(RadiansPerSecond.of(m_sim.get().getVelocityRadPerSec()));

      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.get().getCurrentDrawAmps()));
      mechanismLigament.setAngle(getAngle().in(Degrees));
    }
  }

  /**
   * Get the {@link SmartMotorController} Mechanism Position representing the arm.
   *
   * @return Arm {@link Angle}
   */
  public Angle getAngle()
  {
    return m_motor.getMechanismPosition();
  }

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command setAngle(Angle angle)
  {
    return Commands.run(() -> m_motor.setPosition(angle), m_subsystem);
  }

  /**
   * Set the voltage of the arm.
   *
   * @param volts {@link Voltage} of the Arm to set.
   * @return {@link Command}
   */
  public Command setVoltage(Voltage volts)
  {
    return Commands.run(() -> m_motor.setVoltage(volts), m_subsystem);
  }

  /**
   * Set the dutycycle of the arm motor.
   *
   * @param dutycycle [-1,1] to set.
   * @return {@link Command}
   */
  public Command set(double dutycycle)
  {
    return Commands.run(() -> m_motor.setDutyCycle(dutycycle), m_subsystem);
  }

  /**
   * Arm is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return Trigger on when the arm is near another angle.
   */
  public Trigger isNear(Angle angle, Angle within)
  {
    return new Trigger(() -> getAngle().isNear(angle, within));
  }

  /**
   * Arm is at max, defined by the soft limit or hard limit on the arm.
   *
   * @return Maximum angle for the arm.
   */
  public Trigger max()
  {
    if (m_motor.getConfig().getMechanismUpperLimit().isPresent())
    {
      return new Trigger(gte(m_motor.getConfig().getMechanismUpperLimit().get()));
    }
    if (config.getUpperHardLimit().isEmpty())
    {
      return gte(config.getUpperHardLimit().get());
    }
    throw new IllegalArgumentException("Upper soft and hard limit is empty. Cannot create maximum trigger.");
  }

  /**
   * Minimum angle of the arm given by the soft limit or hard limit of the arm.
   *
   * @return {@link Trigger} on minimum of the arm.
   */
  public Trigger min()
  {
    if (m_motor.getConfig().getMechanismLowerLimit().isPresent())
    {
      return new Trigger(gte(m_motor.getConfig().getMechanismLowerLimit().get()));
    }
    if (config.getLowerHardLimit().isEmpty())
    {
      return gte(config.getLowerHardLimit().get());
    }
    throw new IllegalArgumentException("Lower soft and hard limit is empty. Cannot create maximum trigger.");
  }

  /**
   * Create the SysId routine and commands to run the SysId tests. The SysId test will run the mechanism up then down at
   * a constant speed, then run the mechanism up at an increasing speed and down at an increasing speed. Requires the
   * maximum and minimum limit to be set. Runs the mechanism within 1 degree of the maximum and minimum.
   *
   * @param maximumVoltage Maximum {@link Voltage} to give to the arm, is the voltage given to run the arm up at a
   *                       static speed.
   * @param step           Step {@link Voltage} to give to the arm.
   * @param duration       SysId test duration.
   * @return {@link edu.wpi.first.wpilibj2.command.SequentialCommandGroup} running the SysId commands.
   */
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    SysIdRoutine routine = m_motor.sysId(maximumVoltage, step, duration);
    Angle        max;
    Angle        min;
    if (m_motor.getConfig().getMechanismUpperLimit().isPresent())
    {
      max = m_motor.getConfig().getMechanismUpperLimit().get().minus(Degrees.of(1));
    } else if (config.getUpperHardLimit().isPresent())
    {
      max = config.getUpperHardLimit().get().minus(Degrees.of(1));
    } else
    {
      throw new IllegalArgumentException("No upper soft or hard limit is set. Cannot create SysId command.");
    }
    if (m_motor.getConfig().getMechanismLowerLimit().isPresent())
    {
      min = m_motor.getConfig().getMechanismLowerLimit().get().plus(Degrees.of(1));
    } else if (config.getLowerHardLimit().isPresent())
    {
      min = config.getLowerHardLimit().get().plus(Degrees.of(1));
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
    if (config.getTelemetryName().isPresent())
    {
      group = group.andThen(Commands.print(config.getTelemetryName().get() + " SysId test done."));
    }
    return group;
  }

  /**
   * Between two angles.
   *
   * @param start Start angle.
   * @param end   End angle
   * @return {@link Trigger}
   */
  public Trigger between(Angle start, Angle end)
  {
    return gte(start).and(lte(end));
  }

  /**
   * Less than or equal to angle
   *
   * @param angle {@link Angle} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Angle angle)
  {
    return new Trigger(() -> getAngle().lte(angle));
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return {@link Trigger} for Arm.
   */
  public Trigger gte(Angle angle)
  {
    return new Trigger(() -> getAngle().gte(angle));
  }
}
