package yams.motorcontrollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

public interface SmartMotorController
{

  /**
   * Get the {@link DCMotor} modeling the motor controlled by the motor controller.
   *
   * @return {@link DCMotor} of the controlled motor.
   */
  public DCMotor getDCMotor();

  /**
   * Get the rotations of the motor with the relative encoder since the motor controller powered on scaled to the mechanism rotations.
   * @return {@link Angle} of the relative encoder in the motor scaled to mechanism rotations.
   */
  public Angle getPosition();

  /**
   * Create a {@link SmartMotorController} wrapper from the provided motor controller object.
   *
   * @param motorController Motor controller object.
   * @param motorSim        {@link DCMotor} which the motor controller is connected too.
   * @return {@link SmartMotorController}.
   */
  public static SmartMotorController create(Object motorController, DCMotor motorSim)
  {
    return new TalonFXSWrapper();
  }

}
