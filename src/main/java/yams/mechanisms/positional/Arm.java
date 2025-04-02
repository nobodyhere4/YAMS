package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
   * Get the {@link SmartMotorController} Mechanism Position representing the arm.
   *
   * @return Arm {@link Angle}
   */
  public Angle getPosition()
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
    return Commands.run(() -> m_motor.setPosition(angle));
  }

  public Command sysId(Voltage maximumVoltage, VelocityUnit<VoltageUnit> step, TimeUnit duration)
  {
    return m_motor.sysId(maximumVoltage, step, duration);
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
    return new Trigger(() -> m_motor.getMechanismPosition().lte(angle));
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return {@link Trigger} for Arm.
   */
  public Trigger gte(Angle angle)
  {
    return new Trigger(() -> m_motor.getMechanismPosition().gte(angle));
  }
}
