package yams.motorcontrollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public interface SmartMotorController
{

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


  /**
   * Simulation iteration.
   *
   * @param mechanismVelocity Mechanism velocity to apply to the simulated motor controller.
   */
  public void simIterate(AngularVelocity mechanismVelocity);

  /**
   * Simulation iteration using existing mechanism velocity.
   */
  public void simIterate();

  /**
   * Set the encoder velocity
   *
   * @param velocity {@link AngularVelocity} of the Mechanism.
   */
  public void setEncoderVelocity(AngularVelocity velocity);

  /**
   * Set the encoder velocity.
   *
   * @param velocity Measurement {@link LinearVelocity}
   */
  public void setEncoderVelocity(LinearVelocity velocity);

  /**
   * Set the encoder position
   *
   * @param angle Mechanism {@link Angle} to reach.
   */
  public void setEncoderPosition(Angle angle);

  /**
   * Set the encoder position.
   *
   * @param distance Measurement {@link Distance} to reach.
   */
  public void setEncoderPosition(Distance distance);

  /**
   * Set the Mechanism {@link Angle} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param angle Mechanism angle to set.
   */
  public void setPosition(Angle angle);

  /**
   * Set the Mechanism {@link Distance} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param distance Mechanism {@link Distance} to set.
   */
  public void setPosition(Distance distance);

  /**
   * Set the Mechanism {@link LinearVelocity} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param velocity Mechanism {@link LinearVelocity} to target.
   */
  public void setVelocity(LinearVelocity velocity);

  /**
   * Set the Mechanism {@link AngularVelocity} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param angle Mechanism {@link AngularVelocity} to target.
   */
  public void setVelocity(AngularVelocity angle);

  /**
   * Set the voltage output of the motor controller. Useful for Sysid.
   *
   * @param voltage Voltage to set the motor controller output to.
   */
  public void setVoltage(Voltage voltage);

  /**
   * Set the dutycycle output of the motor controller.
   *
   * @param dutyCycle Value between [-1,1]
   */
  public void setDutyCycle(double dutyCycle);

  /**
   * Run the  {@link edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine} which runs to the maximum MEASUREMENT at the
   * step voltage then down to the minimum MEASUREMENT with the step voltage then up to the maximum MEASUREMENT
   * increasing each second by the step voltage generated via the {@link SmartMotorControllerConfig}.
   *
   * @param maxVoltage   Maximum voltage of the {@link edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine}.
   * @param stepVoltage  Step voltage for the dynamic test in
   *                     {@link edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine}.
   * @param testDuration Duration of each {@link edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine} run.
   * @return Sequential command group of {@link edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine} running all required
   * tests to the configured MINIMUM and MAXIMUM MEASUREMENTS.
   */
  public Command sysId(VoltageUnit maxVoltage, VelocityUnit<VoltageUnit> stepVoltage, TimeUnit testDuration);

  /**
   * Apply the {@link SmartMotorControllerConfig} to the {@link SmartMotorController}.
   *
   * @param config {@link SmartMotorControllerConfig} to use.
   * @return Successful Application of the configuration.
   */
  public boolean applyConfig(SmartMotorControllerConfig config);

  /**
   * Get the duty cycle output of the motor controller.
   *
   * @return DutyCyle of the motor controller.
   */
  public double getDutyCycle();

  /**
   * Get the supply current of the motor controller.
   *
   * @return The supply current of the motor controller.
   */
  public Current getSupplyCurrent();

  /**
   * Get the stator current of the motor controller.
   *
   * @return Stator current
   */
  public Current getStatorCurrent();

  /**
   * Get the voltage output of the motor.
   *
   * @return Voltage output of the motor.
   */
  public Voltage getVoltage();

  /**
   * Get the {@link DCMotor} modeling the motor controlled by the motor controller.
   *
   * @return {@link DCMotor} of the controlled motor.
   */
  public DCMotor getDCMotor();


  /**
   * Get the usable measurement of the motor for mechanisms operating under distance units converted with the
   * {@link SmartMotorControllerConfig}.
   *
   * @return Measurement velocity of the mechanism post-gearing.
   */
  public LinearVelocity getMeasurementVelocity();

  /**
   * Get the usable measurement of the motor for mechanisms operating under distance units converted with the
   * {@link SmartMotorControllerConfig}.
   *
   * @return Measurement velocity of the mechanism post-gearing.
   */
  public Distance getMeasurementPosition();

  /**
   * Get the Mechanism {@link AngularVelocity} taking the configured {@link yams.gearing.MechanismGearing} into the
   * measurement applied via the {@link SmartMotorControllerConfig}.
   *
   * @return Mechanism {@link AngularVelocity}
   */
  public AngularVelocity getMechanismVelocity();

  /**
   * Get the mechanism {@link Angle} taking the configured {@link yams.gearing.MechanismGearing} from
   * {@link SmartMotorControllerConfig}.
   *
   * @return Mechanism {@link Angle}
   */
  public Angle getMechanismPosition();

  /**
   * Gets the angular velocity of the motor.
   *
   * @return {@link AngularVelocity} of the relative motor encoder.
   */
  public AngularVelocity getRotorVelocity();

  /**
   * Get the rotations of the motor with the relative encoder since the motor controller powered on scaled to the
   * mechanism rotations.
   *
   * @return {@link Angle} of the relative encoder in the motor.
   */
  public Angle getRotorPosition();

  /**
   * Update the telemetry under the motor name under the given {@link NetworkTable}
   *
   * @param table {@link NetworkTable} to create the {@link SmartMotorControllerTelemetry} subtable under based off of
   *              {@link SmartMotorControllerConfig#getTelemetryName()}.
   */
  public void updateTelemetry(NetworkTable table);

  /**
   * Update the telemetry under the motor name under the given {@link NetworkTable}
   */
  public void updateTelemetry();

}
