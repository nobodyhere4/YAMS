package yams.motorcontrollers.local;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

public abstract class SparkWrapper implements SmartMotorController
{

  /**
   * Spark motor controller
   */
  private final SparkBase                         spark;
  /**
   * Spark simulation.
   */
  private final SparkSim                          sparkSim;
  /**
   * Spark relative encoder sim object.
   */
  private final SparkRelativeEncoderSim           sparkRelativeEncoderSim;
  /**
   * Spark absolute encoder sim object
   */
  private final Optional<SparkAbsoluteEncoderSim> sparkAbsoluteEncoderSim = Optional.empty();
  /**
   * Motor type.
   */
  private final DCMotor                           motor;
  /**
   * {@link SmartMotorControllerConfig} for the motor.
   */
  private final SmartMotorControllerConfig        config;
  /**
   * Profiled PID controller for the motor controller.
   */
  private       ProfiledPIDController             pidController;

  public SparkWrapper(SparkBase controller, SmartMotorControllerConfig config, DCMotor motor)
  {
    this.motor = motor;
    spark = controller;
    this.config = config;
    pidController = config.getClosedLoopController();
    if (RobotBase.isSimulation())
    {
      sparkSim = new SparkSim(spark, motor);
      if (spark instanceof SparkMax)
      {
        sparkRelativeEncoderSim = new SparkRelativeEncoderSim((SparkMax) spark);
      } else if (spark instanceof SparkFlex)
      {
        sparkRelativeEncoderSim = new SparkRelativeEncoderSim((SparkFlex) spark);
      } else
      {
        sparkRelativeEncoderSim = null;
      }
    } else
    {

      sparkSim = null;
      sparkRelativeEncoderSim = null;
    }
  }

  /**
   * Apply the PID to the motor.
   */
  public abstract void applyPIDController();


  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {
    if (RobotBase.isSimulation())
    {
      sparkSim.iterate(mechanismVelocity.in(RotationsPerSecond),
                       RoboRioSim.getVInVoltage(),
                       Milliseconds.of(20).in(Second));
      sparkRelativeEncoderSim.iterate(mechanismVelocity.in(RotationsPerSecond), Milliseconds.of(20).in(Seconds));
      sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.iterate(mechanismVelocity.in(
                                                                                             RotationsPerSecond),
                                                                                         Milliseconds.of(20)
                                                                                                     .in(Seconds)));
    }
  }

  @Override
  public void setVelocity(AngularVelocity velocity)
  {

  }

  @Override
  public void setVelocity(LinearVelocity velocity)
  {

  }

  @Override
  public void setPosition(Angle angle)
  {

  }

  @Override
  public void setPosition(Distance distance)
  {

  }

  @Override
  public Command sysId(VoltageUnit maxVoltage, VelocityUnit<VoltageUnit> stepVoltage, TimeUnit testDuration)
  {
    return null;
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    return false;
  }

  @Override
  public double getDutyCycle()
  {
    return 0;
  }

  @Override
  public Current getSupplyCurrent()
  {
    return null;
  }

  @Override
  public Current getStatorCurrent()
  {
    return null;
  }

  @Override
  public Voltage getVoltage()
  {
    return null;
  }

  @Override
  public DCMotor getDCMotor()
  {
    return null;
  }

  @Override
  public LinearVelocity getMeasurementVelocity()
  {
    return null;
  }

  @Override
  public Distance getMeasurementPosition()
  {
    return null;
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    return null;
  }

  @Override
  public Angle getMechanismPosition()
  {
    return null;
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return null;
  }

  @Override
  public Angle getRotorPosition()
  {
    return null;
  }
}
