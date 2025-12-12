package frc.robot.subsystems.doubleflywheel;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * With 2 {@link SmartMotorController}s we can control a DoubleFlyWheel which can accurately control arc of an object
 * given tuning data.
 *
 * @implNote Because both {@link SmartMotorController}s are in this Subsystem any Live Tuning or SysId will result in no
 * control being given to the other SmartMotorController, which will result in undefined behavior!
 */
public class DoubleFlyWheelSubsystem extends SubsystemBase
{

  // Special note: You cannot run SysId on either of the SmartMotorControllers because they both use the same subsystem and undefined behavior can happen on the one not running sysId.
  private SmartMotorControllerConfig upperFlyWheelConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withIdleMode(MotorMode.COAST)
      .withWheelDiameter(Inches.of(4)) // Only needed to find the MPH of the flywheel for fun.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMomentOfInertia(Inches.of(4), Pounds.of(2))
      .withClosedLoopController(1,
                                0,
                                0) // You generally do not want a profile because its not a position controlled loop.
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // Helps track changing RPM goals
      .withMotorInverted(false);

  private SmartMotorControllerConfig lowerFlyWheelConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withIdleMode(MotorMode.COAST)
      .withWheelDiameter(Inches.of(4)) // Only needed to find the MPH of the flywheel for fun.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMomentOfInertia(Inches.of(4), Pounds.of(2))
      .withClosedLoopController(1,
                                0,
                                0) // You generally do not want a profile because its not a position controlled loop.
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // Helps track changing RPM goals
      .withMotorInverted(false);


  private SmartMotorController upperFlyWheel = new TalonFXWrapper(new TalonFX(6),
                                                                  DCMotor.getKrakenX60(1),
                                                                  upperFlyWheelConfig);
  private SmartMotorController lowerFlyWheel = new TalonFXWrapper(new TalonFX(4),
                                                                  DCMotor.getKrakenX60(1),
                                                                  lowerFlyWheelConfig);


  public DoubleFlyWheelSubsystem()
  {

  }

  /**
   * Get the linear speed (commonly used for KM/h or MPH) of the flywheels.
   *
   * @return (Lower Flywheel Speed, Upper Flywheel Speed)
   */
  public Pair<LinearVelocity, LinearVelocity> getLinearSpeed()
  {
    return Pair.of(lowerFlyWheel.getMeasurementVelocity(), upperFlyWheel.getMeasurementVelocity());
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
    return startRun(() -> {
      // Stop the closed loop controller to prevent the flywheels from operating under the closed loop controller.
      upperFlyWheel.stopClosedLoopController();
      lowerFlyWheel.stopClosedLoopController();
    }, () -> {
      upperFlyWheel.setDutyCycle(upper);
      lowerFlyWheel.setDutyCycle(lower);
    }).finallyDo(() -> {
      // Start the closed loop controller to allow the flywheels to operate under the closed loop controller.
      upperFlyWheel.startClosedLoopController();
      lowerFlyWheel.startClosedLoopController();
    });
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
    return startRun(() -> {
      // Stop the closed loop controller to prevent the flywheels from operating under the closed loop controller.
      upperFlyWheel.stopClosedLoopController();
      lowerFlyWheel.stopClosedLoopController();
    }, () -> {
      upperFlyWheel.setVoltage(upper);
      lowerFlyWheel.setVoltage(lower);
    }).finallyDo(() -> {
      // Start the closed loop controller to allow the flywheels to operate under the closed loop controller.
      upperFlyWheel.startClosedLoopController();
      lowerFlyWheel.startClosedLoopController();
    });
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
    return run(() -> {
      var lowerSpeeds = RPM.of(DoubleFlyWheelConstants.distanceToRPM.getFirst().get(distanceToGoal.get().in(Meters)));
      var upperSpeeds = RPM.of(DoubleFlyWheelConstants.distanceToRPM.getSecond().get(distanceToGoal.get().in(Meters)));
      lowerFlyWheel.setVelocity(lowerSpeeds);
      upperFlyWheel.setVelocity(upperSpeeds);
    }).withName("Set Speed For Distance");
  }

}

