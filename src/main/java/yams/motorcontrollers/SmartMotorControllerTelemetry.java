package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;

public class SmartMotorControllerTelemetry
{

  /**
   * Mechanism lower limit reached.
   */
  public boolean mechanismLowerLimit = false;
  /**
   * Mechanism upper limit reached.
   */
  public boolean mechanismUpperLimit = false;
  /**
   * Motor temperature cutoff reached.
   */
  public boolean temperatureLimit    = false;
  /**
   * Velocity PID controller used.
   */
  public boolean velocityControl     = false;
  /**
   * Elevator feedforward used.
   */
  public boolean elevatorFeedforward = false;
  /**
   * Arm feedforward used.
   */
  public boolean armFeedforward      = false;
  /**
   * Simple feedforward used.
   */
  public boolean simpleFeedforward   = false;
  /**
   * Motion profiling used.
   */
  public boolean motionProfile       = false;

  /**
   * Setpoint position given.
   */
  public double setpointPosition = 0;
  /**
   * Setpoint velocity given.
   */
  public double setpointVelocity = 0;

  /**
   * Feedforward voltage supplied to the {@link SmartMotorController}
   */
  public double feedforwardVoltage = 0.0;
  /**
   * PID Output voltage supplied to the {@link SmartMotorController}
   */
  public double pidOutputVoltage   = 0.0;
  /**
   * Output voltage to the {@link SmartMotorController}
   */
  public double outputVoltage      = 0.0;

  /**
   * Stator current (motor controller output current) to the Motor.
   */
  public double      statorCurrent = 0.0;
  /**
   * Motor temperature.
   */
  public Temperature temperature;

  /**
   * Mechanism distance.
   */
  public Distance        distance       = Meters.of(0);
  /**
   * Mechanism linear velocity.
   */
  public LinearVelocity  linearVelocity = MetersPerSecond.of(0);
  /**
   * Mechanism position.
   */
  public Angle           mechanismPosition;
  /**
   * Mechanism velocity.
   */
  public AngularVelocity mechanismVelocity;
  /**
   * Rotor position.
   */
  public Angle           rotorPosition;
  /**
   * Rotor velocity.
   */
  public AngularVelocity rotorVelocity;
}
