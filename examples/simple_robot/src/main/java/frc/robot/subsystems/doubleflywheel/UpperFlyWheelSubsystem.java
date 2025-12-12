package frc.robot.subsystems.doubleflywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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
 * Upper Flywheel subsystem, separated to allow for live tuning and sysId.
 */
public class UpperFlyWheelSubsystem extends SubsystemBase
{

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
      .withMotorInverted(false)
      .withTelemetry("UpperFlyWheel", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
  private SmartMotorController       flyWheel            = new TalonFXWrapper(new TalonFX(6),
                                                                              DCMotor.getKrakenX60(1),
                                                                              upperFlyWheelConfig);


  /**
   * Get the LinearVelocity to convert to MPH or KMH.
   *
   * @return {@link LinearVelocity} of the flywheel.
   */
  public LinearVelocity getLinearVelocity()
  {
    return flyWheel.getMeasurementVelocity();
  }

  /**
   * Get the AngularVelocity of the flywheel.
   *
   * @return {@link AngularVelocity} of the flywheel.
   */
  public AngularVelocity getAngularVelocity()
  {
    return flyWheel.getMechanismVelocity();
  }

  /**
   * Sets the duty cycle of the flywheel.
   *
   * @param dutycycle Duty cycle to set. [-1, 1]
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setDutyCycle(double dutycycle)
  {
    // Stop the closed loop controller to prevent the flywheels from operating under the closed loop controller.
    return startRun(() -> flyWheel.stopClosedLoopController(),
                    () -> flyWheel.setDutyCycle(dutycycle))
        // Start the closed loop controller to allow the flywheels to operate under the closed loop controller.
        .finallyDo(() -> flyWheel.startClosedLoopController())
        .withName("Duty Cycle");
  }

  /**
   * Sets the voltage of the flywheel.
   *
   * @param volts Voltage to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVoltage(Voltage volts)
  {
    // Stop the closed loop controller to prevent the flywheels from operating under the closed loop controller.
    return startRun(() -> flyWheel.stopClosedLoopController(),
                    () -> flyWheel.setVoltage(volts))
        // Start the closed loop controller to allow the flywheels to operate under the closed loop controller.
        .finallyDo(() -> flyWheel.startClosedLoopController())
        .withName("Voltage");
  }

  /**
   * Set the velocity of the flywheel.
   *
   * @param velocity Velocity to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity velocity)
  {
    return run(() -> flyWheel.setVelocity(velocity)).withName("Velocity");
  }

  /**
   * Set the velocity of the flywheel.
   *
   * @param velocitySupplier Velocity supplier.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(Supplier<AngularVelocity> velocitySupplier)
  {
    return run(() -> flyWheel.setVelocity(velocitySupplier.get())).withName("Velocity Supplier");
  }

  /**
   * Create a {@link edu.wpi.first.wpilibj2.command.RunCommand} to set the speeds for the upper and lower flywheels
   * based off the distance to the goal.
   *
   * @param distanceSupplier Distance from the center of the robot to the goal on the XY plane.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setDistance(Supplier<Distance> distanceSupplier)
  {
    return run(() -> {
      var upperSpeeds = RPM.of(DoubleFlyWheelConstants.distanceToRPM.getSecond()
                                                                    .get(distanceSupplier.get().in(Meters)));
      flyWheel.setVelocity(upperSpeeds);
    }).withName("Distance to RPM");
  }

  @Override
  public void periodic()
  {
    flyWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    flyWheel.simIterate();
  }
}
