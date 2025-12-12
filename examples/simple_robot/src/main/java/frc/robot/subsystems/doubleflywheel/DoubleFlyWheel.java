package frc.robot.subsystems.doubleflywheel;


import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import yams.motorcontrollers.SmartMotorController;

/**
 * With 2 {@link SmartMotorController}s we can control a DoubleFlyWheel which can accurately control arc of an object
 * given tuning data.
 *
 */
public class DoubleFlyWheel
{

  private LowerFlyWheelSubsystem lowerFlyWheelSubsystem = new LowerFlyWheelSubsystem();
  private UpperFlyWheelSubsystem upperFlyWheelSubsystem = new UpperFlyWheelSubsystem();

  /**
   * Get the linear speed (commonly used for KM/h or MPH) of the flywheels.
   *
   * @return (Lower Flywheel Speed, Upper Flywheel Speed)
   */
  public Pair<LinearVelocity, LinearVelocity> getLinearSpeed()
  {
    return Pair.of(lowerFlyWheelSubsystem.getLinearVelocity(), upperFlyWheelSubsystem.getLinearVelocity());
  }

  /**
   * Set the duty cycle of the upper and lower flywheels.
   *
   * @param lower Lower duty cycle.
   * @param upper Upper duty cycle.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setDutyCycle(double lower, double upper)
  {
    return Commands.parallel(lowerFlyWheelSubsystem.setDutyCycle(lower), upperFlyWheelSubsystem.setDutyCycle(upper))
                   .withName("Set Duty Cycle (Double FlyWheel)");
  }

  /**
   * Set the voltage of the upper and lower flywheels.
   *
   * @param lower Lower voltage.
   * @param upper Upper voltage.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVoltage(Voltage lower, Voltage upper)
  {
    return Commands.parallel(lowerFlyWheelSubsystem.setVoltage(lower), upperFlyWheelSubsystem.setVoltage(upper))
                   .withName("Set Voltage (Double FlyWheel)");
  }

  /**
   * Create a {@link edu.wpi.first.wpilibj2.command.RunCommand} to set the speeds for the upper and lower flywheels
   * based off the distance to the goal.
   *
   * @param distanceToGoal Distance from the center of the robot to the goal on the XY plane.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setSpeedForDistance(Supplier<Distance> distanceToGoal)
  {
    return Commands.parallel(lowerFlyWheelSubsystem.setDistance(distanceToGoal),
                             upperFlyWheelSubsystem.setDistance(distanceToGoal))
                   .withName("Set Speed For Distance (Double FlyWheel)");
  }

}

