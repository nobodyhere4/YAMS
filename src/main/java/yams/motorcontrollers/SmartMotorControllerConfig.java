package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
import yams.gearing.MechanismGearing;

/**
 * Smart motor controller config.
 */
public class SmartMotorControllerConfig
{

  /**
   * External encoder.
   */
  private Object                           externalEncoder;
  /**
   * Follower motors
   */
  private Object[]                         followers;
  /**
   * Simple feedforward for the motor controller.
   */
  private Optional<SimpleMotorFeedforward> simpleFeedforward        = Optional.empty();
  /**
   * Elevator feedforward for the motor controller.
   */
  private Optional<ElevatorFeedforward>    elevatorFeedforward      = Optional.empty();
  /**
   * Arm feedforward for the motor controller.
   */
  private Optional<ArmFeedforward>         armFeedforward           = Optional.empty();
  /**
   * Controller for the {@link SmartMotorController}.
   */
  private ProfiledPIDController            controller;
  /**
   * Gearing for the {@link SmartMotorController}.
   */
  private MechanismGearing                 gearing;
  /**
   * Measurement ratio to convert from mechanism rotations to usable measurements. DISTANCE/ROTATIONS
   */
  private double                           mechanismToDistanceRatio = 1.0;
  /**
   * PID Controller period for robot controller based PIDs
   */
  private Time                             controlPeriod            = Milliseconds.of(20);
  /**
   * Open loop ramp rate.
   */
  private double                           openLoopRampRate         = 1.0;
  /**
   * Closed loop ramp rate.
   */
  private double                           closeLoopRampRate        = 1.0;
  /**
   * Set the stator current limit in Amps for the {@link SmartMotorController}
   */
  private double                           statorStallCurrentLimit;
  /**
   * The supply current limit in Amps for the {@link SmartMotorController}
   */
  private double                           supplyStallCurrentLimit;
  /**
   * The voltage compensation.
   */
  private double                           voltageCompensation;
  /**
   * Set the {@link MotorMode} for the {@link SmartMotorController}.
   */
  private MotorMode                        idleMode;
  /**
   * Low distance soft limit.
   */
  private Distance                         lowDistanceLimit;
  /**
   * High distance soft limit.
   */
  private Distance                         highDistanceLimit;
  /**
   * Low angle soft limit.
   */
  private Angle                            lowAngleLimit;
  /**
   * High angle soft limit.
   */
  private Angle                            highAngleLimit;
  /**
   * Name for the {@link SmartMotorController} telemetry.
   */
  private String                           telemetryName;
  /**
   * Telemetry verbosity setting.
   */
  private TelemetryVerbosity               verbosity;

  /**
   * Set the telemetry for the {@link SmartMotorController}
   *
   * @param telemetryName Name for the {@link SmartMotorController}
   * @param verbosity     Verbosity of the Telemetry for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withTelemetry(String telemetryName, TelemetryVerbosity verbosity)
  {
    this.telemetryName = telemetryName;
    this.verbosity = verbosity;
    return this;
  }

  /**
   * Get the stator stall current limit.
   *
   * @return Stator current limit.
   */
  public double getStatorStallCurrentLimit()
  {
    return statorStallCurrentLimit;
  }

  /**
   * Set the distance soft limits.
   *
   * @param low  Low distance soft limit.
   * @param high High distance soft limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withSoftLimit(Distance low, Distance high)
  {
    if (mechanismToDistanceRatio == 1.0)
    {
      throw new IllegalArgumentException(
          "Mechanism to distance ratio is not defined! Please configure it with 'SmartMotionControllerConfig.withMechanismToDistanceRatio()'");
    }
    lowDistanceLimit = low;
    highDistanceLimit = high;
    return this;
  }

  /**
   * Set the angle soft limits.
   *
   * @param low  Low angle soft limit.
   * @param high High angle soft limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withSoftLimit(Angle low, Angle high)
  {
    lowAngleLimit = low;
    highAngleLimit = high;
    return this;
  }

  /**
   * Get the supply stall current limit.
   *
   * @return Supply stall current limit.
   */
  public double getSupplyStallCurrentLimit()
  {
    return supplyStallCurrentLimit;
  }

  /**
   * Get the voltage compensation for the {@link SmartMotorController}
   *
   * @return Ideal voltage
   */
  public double getVoltageCompensation()
  {
    return voltageCompensation;
  }

  /**
   * Get the idle mode for the {@link SmartMotorController}
   *
   * @return {@link MotorMode}
   */
  public MotorMode getIdleMode()
  {
    return idleMode;
  }

  /**
   * Low distance soft limit.
   *
   * @return Lower distance soft limit.
   */
  public Distance getLowDistanceLimit()
  {
    return lowDistanceLimit;
  }

  /**
   * High distance soft limit.
   *
   * @return Higher distance soft limit.
   */
  public Distance getHighDistanceLimit()
  {
    return highDistanceLimit;
  }

  /**
   * Low angle soft limit.
   *
   * @return Lower angle soft limit.
   */
  public Angle getLowAngleLimit()
  {
    return lowAngleLimit;
  }

  /**
   * High angle soft limit.
   *
   * @return Higher angle soft limit.
   */
  public Angle getHighAngleLimit()
  {
    return highAngleLimit;
  }

  /**
   * Set the {@link SmartMotorController} to brake or coast mode.
   *
   * @param idleMode {@link MotorMode} idle mode
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withIdleMode(MotorMode idleMode)
  {
    this.idleMode = idleMode;
    return this;
  }

  /**
   * Set the voltage compensation for the {@link SmartMotorController}
   *
   * @param voltageCompensation Ideal voltage value.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withVoltageCompensation(double voltageCompensation)
  {
    this.voltageCompensation = voltageCompensation;
    return this;
  }

  /**
   * Set the follower motors of the {@link SmartMotorController}
   *
   * @param followers Base motor types (NOT {@link SmartMotorController}!) to configure as followers, must be same brand
   *                  as the {@link SmartMotorController}.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  public SmartMotorControllerConfig withFollowers(Object... followers)
  {
    this.followers = followers;
    return this;
  }

  /**
   * Set the stall stator current limit for the {@link SmartMotorController}
   *
   * @param stallCurrent Stall stator current limit for the {@link SmartMotorController}.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withStatorCurrentLimit(Current stallCurrent)
  {
    this.statorStallCurrentLimit = stallCurrent.in(Amps);
    return this;
  }

  /**
   * Set the stall supply current limit for the {@link SmartMotorController}
   *
   * @param supplyCurrent Supply current limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withSupplyCurrentLimit(Current supplyCurrent)
  {
    this.supplyStallCurrentLimit = supplyCurrent.in(Amps);
    return this;
  }

  /**
   * Set the closed loop ramp rate. The ramp rate is the minimum time in seconds it should take to go from 0 power to
   * full power in the motor controller while using PID.
   *
   * @param rate closed loop ramp rate.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopRampRate(double rate)
  {
    this.closeLoopRampRate = rate;
    return this;
  }

  /**
   * Set the open loop ramp rate. The ramp rate is the minimum time in seconds it should take to go from 0 power to full
   * power in the motor controller while not using PID.
   *
   * @param rate open loop ramp rate.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withOpenLoopRampRate(double rate)
  {
    this.openLoopRampRate = rate;
    return this;
  }

  /**
   * Set the external encoder which is attached to the motor type sent used by {@link SmartMotorController}
   *
   * @param externalEncoder External encoder attached to the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withExternalEncoder(Object externalEncoder)
  {
    this.externalEncoder = externalEncoder;
    return this;
  }

  /**
   * Get the follower motors to the {@link SmartMotorControllerConfig}
   *
   * @return Follower motor list.
   */
  public Object[] getFollowers()
  {
    return followers;
  }

  /**
   * Set the {@link MechanismGearing} for the {@link SmartMotorController}.
   *
   * @param gear {@link MechanismGearing} representing the gearbox and sprockets to the final axis.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withGearing(MechanismGearing gear)
  {
    gearing = gear;
    return this;
  }

  /**
   * Set the conversion factor to transform the rotations into distance of the mechanism.
   *
   * @param ratio DISTANCE/ROTATIONS
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withMechanismToDistanceRatio(double ratio)
  {
    mechanismToDistanceRatio = ratio;
    return this;
  }

  /**
   * Get the mechanism to distance ratio for the {@link SmartMotorController}
   *
   * @return Rotations/Distance ratio to convert mechanism rotations to distance.
   */
  public double getMechanismToDistanceRatio()
  {
    return mechanismToDistanceRatio;
  }

  /**
   * Modify the period of the PID controller for the motor controller.
   *
   * @param time Period of the motor controller PID.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withPeriod(Time time)
  {
    controlPeriod = time;
    return this;
  }

  /**
   * Get the {@link ArmFeedforward} if it is set.
   *
   * @return {@link Optional} of the {@link ArmFeedforward}.
   */
  public Optional<ArmFeedforward> getArmFeedforward()
  {
    return armFeedforward;
  }

  /**
   * Configure the {@link ArmFeedforward} for the
   *
   * @param armFeedforward Arm feedforward for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withFeedforward(ArmFeedforward armFeedforward)
  {
    if (armFeedforward == null)
    {
      this.armFeedforward = Optional.empty();
    } else
    {
      this.elevatorFeedforward = Optional.empty();
      this.simpleFeedforward = Optional.empty();
      this.armFeedforward = Optional.of(armFeedforward);
    }
    return this;
  }

  /**
   * Get the {@link ElevatorFeedforward} {@link Optional}
   *
   * @return {@link ElevatorFeedforward} {@link Optional}
   */
  public Optional<ElevatorFeedforward> getElevatorFeedforward()
  {
    return elevatorFeedforward;
  }

  /**
   * Configure {@link ElevatorFeedforward} for the {@link SmartMotorController}
   *
   * @param elevatorFeedforward {@link ElevatorFeedforward} to set.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withFeedforward(ElevatorFeedforward elevatorFeedforward)
  {
    if (elevatorFeedforward == null)
    {
      this.elevatorFeedforward = Optional.empty();
    } else
    {
      this.armFeedforward = Optional.empty();
      this.simpleFeedforward = Optional.empty();
      this.elevatorFeedforward = Optional.of(elevatorFeedforward);
    }
    return this;
  }

  /**
   * Get the {@link SimpleMotorFeedforward} {@link Optional}.
   *
   * @return {@link SimpleMotorFeedforward} {@link Optional}
   */
  public Optional<SimpleMotorFeedforward> getSimpleFeedforward()
  {
    return simpleFeedforward;
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}
   *
   * @param controller {@link ProfiledPIDController} to use.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(ProfiledPIDController controller)
  {
    this.controller = controller;
    return this;
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}. Units are Meters.
   *
   * @param kP              KP scalar for the PID Controller.
   * @param kI              KI scalar for the PID Controller.
   * @param kD              KD scalar for the PID Controller.
   * @param maxVelocity     Maximum linear velocity for the Trapazoidal profile.
   * @param maxAcceleration Maximum linear acceleration for the Trapazoidal profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(double kP, double kI, double kD,
                                                             LinearVelocity maxVelocity,
                                                             LinearAcceleration maxAcceleration)
  {
    if (mechanismToDistanceRatio == 1.0)
    {
      throw new IllegalArgumentException(
          "Measurement ratio must be defined. Please configure the measurement ratio using 'SmartMotorControllerConfig.withMechanismToDistanceRatio()'");
    }
    this.controller = new ProfiledPIDController(kP,
                                                kI,
                                                kD,
                                                new Constraints(maxVelocity.in(MetersPerSecond),
                                                                maxAcceleration.in(MetersPerSecondPerSecond)));
    return this;
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}. Units are Meters.
   *
   * @param kP              KP scalar for the PID Controller.
   * @param kI              KI scalar for the PID Controller.
   * @param kD              KD scalar for the PID Controller.
   * @param maxVelocity     Maximum angular velocity for the Trapazoidal profile.
   * @param maxAcceleration Maximum angular acceleration for the Trapazoidal profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(double kP, double kI, double kD,
                                                             AngularVelocity maxVelocity,
                                                             AngularAcceleration maxAcceleration)
  {
    this.controller = new ProfiledPIDController(kP,
                                                kI,
                                                kD,
                                                new Constraints(maxVelocity.in(RotationsPerSecond),
                                                                maxAcceleration.in(RotationsPerSecondPerSecond)));
    return this;
  }

  /**
   * Get the controller for the {@link SmartMotorController}
   *
   * @return {@link ProfiledPIDController}
   */
  public ProfiledPIDController getClosedLoopController()
  {
    return controller;
  }

  /**
   * Set the {@link SimpleMotorFeedforward} for {@link SmartMotorController}
   *
   * @param simpleFeedforward {@link SimpleMotorFeedforward}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withFeedforward(SimpleMotorFeedforward simpleFeedforward)
  {
    if (simpleFeedforward == null)
    {
      this.simpleFeedforward = Optional.empty();
    } else
    {
      this.armFeedforward = Optional.empty();
      this.elevatorFeedforward = Optional.empty();
      this.simpleFeedforward = Optional.of(simpleFeedforward);
    }
    return this;
  }

  /**
   * Get the period of the {@link SmartMotorController} closed loop period.
   *
   * @return {@link SmartMotorController} closed loop controller period.
   */
  public Time getClosedLoopControlPeriod()
  {
    return controlPeriod;
  }

  /**
   * Get the gearing to convert rotor rotations to mechanisms rotations connected to the {@link SmartMotorController}
   *
   * @return {@link MechanismGearing} representing the gearbox and sprockets attached to the
   * {@link SmartMotorController}.
   */
  public MechanismGearing getGearing()
  {
    return gearing;
  }

  /**
   * Get the external encoder.
   *
   * @return Attached external encoder.
   */
  public Object getExternalEncoder()
  {
    return externalEncoder;
  }

  /**
   * Get the open loop ramp rate.
   *
   * @return Open loop ramp rate.
   */
  public double getOpenLoopRampRate()
  {
    return openLoopRampRate;
  }

  /**
   * Get the closed loop ramp rate.
   *
   * @return Closed loop ramp.
   */
  public double getClosedLoopRampRate()
  {
    return closeLoopRampRate;
  }

  /**
   * Telemetry verbosity for the {@link SmartMotorController}
   */
  public enum TelemetryVerbosity
  {
    LOW, MID, HIGH
  }

  /**
   * Idle mode for the {@link SmartMotorController}
   */
  public enum MotorMode
  {
    /**
     * Brake mode.
     */
    BRAKE,
    /**
     * Coast mode.
     */
    COAST
  }
}
