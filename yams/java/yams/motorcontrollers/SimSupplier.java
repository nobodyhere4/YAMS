package yams.motorcontrollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Provides sim functions for
 */
public interface SimSupplier
{

  /**
   * Gets the supply voltage for the motor controller.
   *
   * @return Supply voltage to the motor controller
   */
  public Voltage getMechanismSupplyVoltage();

  /**
   * Get the mechanism position.
   *
   * @return mechanism angle.
   */
  public Angle getMechanismPosition();

  /**
   * Set the Mechanism position
   *
   * @param position Position of the mechanism.
   */
  public void setMechanismPosition(Angle position);

  /**
   * Get the rotor position.
   *
   * @return rotor position.
   */
  public Angle getRotorPosition();

  /**
   * Get the mechanism velocity.
   *
   * @return Mechanism velocity.
   */
  public AngularVelocity getMechanismVelocity();

  /**
   * Set the Mechanism velocity.
   *
   * @param velocity Mechanism velocity.
   */
  public void setMechanismVelocity(AngularVelocity velocity);

  /**
   * Get the rotor velocity.
   *
   * @return rotor velocity.
   */
  public AngularVelocity getRotorVelocity();
}
