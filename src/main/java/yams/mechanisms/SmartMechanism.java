package yams.mechanisms;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import javax.swing.SpringLayout.Constraints;
import yams.motorcontrollers.SmartMotorController;

/**
 * Generic implementation of a mechanism with advanced telemetry.
 */
public abstract class SmartMechanism
{

  /**
   * SysId routine to tune the mechanism.
   */
  protected Optional<SysIdRoutine>          m_sysIdRoutine    = Optional.empty();
  /**
   * Motor controller for the primary motor in the mechanism.
   */
  protected Optional<SmartMotorController>  m_motor = Optional.empty();
  /**
   * Name of the mechanism for command decoration and telemetry.
   */
  protected Optional<String>                m_name = Optional.empty();
  /**
   * PID controller for the mechanism.
   */
  protected Optional<ProfiledPIDController> m_pidController = Optional.empty();
  /**
   * Mechanism {@link NetworkTable}
   */
  protected NetworkTable                    m_networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  /**
   * Boolean publisher that is true when the {@link SmartMotorController} is experiencing problems or does not exist.
   */
  protected BooleanPublisher m_motorError;
}
