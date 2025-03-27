package yams.mechanisms.positional;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import yams.mechanisms.SmartMechanism;

/**
 * Generic for positional mechanisms.
 */
public abstract class SmartPositionalMechanism extends SmartMechanism
{

  /**
   * Trigger when the positional mechanism reaches its maximum position.
   */
  public    Optional<Trigger> atMax = Optional.empty();
  /**
   * Trigger when the positional mechanism reaches it minimum position.
   */
  public    Optional<Trigger> atMin = Optional.empty();
  /**
   * Setpoint for the positional mechanism.
   */
  protected double            m_setpoint;
  /**
   * Network Tables publisher for the units displayed by the positional mechanism.
   */
  protected StringPublisher   m_units;
  /**
   * Network Tables Publisher for the setpoint.
   */
  protected DoublePublisher   m_setpointPublisher;
  /**
   * Network Tables publisher for the position of the mechanism.
   */
  protected DoublePublisher   m_positionPublisher;
  /**
   * Network Tables publisher for the externally detected position of the mechanism.
   */
  protected DoublePublisher   m_externalPositionPublisher;


}
