package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Optional;
import yams.exceptions.MotorNotPresentException;
import yams.sensors.absoluteencoders.SmartAbsoluteEncoder;

public class Arm extends SmartPositionalMechanism
{

  /**
   * Absolute encoder for the arm.
   */
  private Optional<SmartAbsoluteEncoder> m_absoluteEncoder      = Optional.empty();
  /**
   * Feedforward for the arm.
   */
  private Optional<ArmFeedforward>       m_feedforward          = Optional.empty();
  /**
   * Simulation for the arm.
   */
  private Optional<SingleJointedArmSim>  m_sim                  = Optional.empty();
  /**
   * Angle at which the absolute encoder reads 0 while the arm is horizontal.
   */
  private Optional<Angle>                m_horizantalZeroOffset = Optional.empty();
  /**
   * Setpoint angle for the arm. 0 is horizontal.
   */
  private Angle                          m_setpointAngle;

  /**
   * Update the mechanism's telemetry.
   */
  public void updateTelemetry()
  {
    if (m_name.isPresent())
    {
      m_setpointPublisher.set(m_setpointAngle.in(Degrees));
      m_positionPublisher.set(getPosition().in(Degrees));
    }
  }

  public Angle getPosition()
  {
    if (m_motor.isPresent())
    {
      return m_motor.get().getPosition();
    }
    throw new MotorNotPresentException("Arm");
  }

}
