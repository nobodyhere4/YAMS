package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.Supplier;
import yams.exceptions.ArmConfigurationException;
import yams.exceptions.DoubleJointedArmConfigurationException;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.simulation.ArmSimSupplier;

/**
 * Arm mechanism.
 */
public class DoubleJointedArm extends SmartPositionalMechanism
{

  /**
   * Upper arm {@link SmartMotorController}
   */
  private final SmartMotorController          m_upperSMC;
  /**
   * Lower arm {@link SmartMotorController}
   */
  private final SmartMotorController          m_lowerSMC;
  /**
   * Arm config.
   */
  private final ArmConfig                     m_lowerArmConfig;
  /**
   * Arm config.
   */
  private final ArmConfig                     m_upperArmConfig;
  /**
   * Simulation for the arm.
   */
  private       Optional<SingleJointedArmSim> m_lowerArmSim = Optional.empty();
  /**
   * Simulation for the arm.
   */
  private       Optional<SingleJointedArmSim> m_upperArmSim = Optional.empty();
  /**
   * Lower ligament
   */
  private       MechanismLigament2d           m_lowerLigament;
  /**
   * Upper root
   */
  private       MechanismRoot2d               m_upperRoot;
  /**
   * Upper ligament.
   */
  private       MechanismLigament2d           m_upperLigament;
  /**
   * Lower Arm root position in meters.
   */
  private       Translation2d                 m_lowerArmRootPos;
  /**
   * Lower arm length used for trig calculations on current position.
   */
  private       Distance                      m_lowerArmLength;
  /**
   * Upper arm length used for trig calculations on current position.
   */
  private       Distance                      m_upperArmLength;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param lowerConfig Lower {@link ArmConfig} to use.
   */
  public DoubleJointedArm(ArmConfig lowerConfig, ArmConfig upperConfig)
  {
    m_lowerArmConfig = lowerConfig;
    m_upperArmConfig = upperConfig;
    m_lowerSMC = lowerConfig.getMotor();
    m_upperSMC = upperConfig.getMotor();
    if (lowerConfig.getMotor().getConfig().getSubsystem() != upperConfig.getMotor().getConfig().getSubsystem())
    {
      throw new DoubleJointedArmConfigurationException("SmartMotorControllers do not have the same subsystem!",
                                                       "Cannot create commands for single subsystem.",
                                                       "withSubsystem(this)");
    }

    // Check that the starting angle is defined
    if (lowerConfig.getStartingAngle().isEmpty() || upperConfig.getStartingAngle().isEmpty())
    {
      throw new DoubleJointedArmConfigurationException("Arm starting angle is empty",
                                                       "Cannot create simulation.",
                                                       "withStartingPosition(Angle)");
    }

    // Check that the arm lengths are defined
    if (lowerConfig.getLength().isEmpty() || upperConfig.getLength().isEmpty())
    {
      throw new DoubleJointedArmConfigurationException(
          "Arm legnths must be defined to calculate current end position of the Double Jointed Arm!",
          "Cannot create mechanism",
          "withLength(Distance)");
    }
    m_lowerArmLength = lowerConfig.getLength().get();
    m_upperArmLength = upperConfig.getLength().get();

    // Setup root mechanism position for calculations.
    var lowerMechPosCfg = lowerConfig.getMechanismPositionConfig();
//    var lowerMechanismX = lowerMechPosCfg.getMechanismX(m_lowerArmLength).in(Meters);

    var upperMechPosCfg = upperConfig.getMechanismPositionConfig();
//    var upperMechanismX = upperMechPosCfg.getMechanismX(m_upperArmLength).in(Meters);
    m_lowerArmRootPos = new Translation2d(m_lowerArmLength.plus(m_upperArmLength).in(Meters), 0);

    // Seed the relative encoder
    m_lowerSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_lowerSMC.seedRelativeEncoder();
    });
    m_upperSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_upperSMC.seedRelativeEncoder();
    });

    // Setup telemetry
    lowerConfig.getTelemetryName().ifPresent(name -> {
      m_telemetry.setupTelemetry(getName() + "/lower", m_lowerSMC);
    });
    upperConfig.getTelemetryName().ifPresent(name -> {
      m_telemetry.setupTelemetry(getName() + "/upper", m_upperSMC);
    });

    if (RobotBase.isSimulation())
    {

      if (lowerConfig.getLowerHardLimit().isEmpty() || upperConfig.getLowerHardLimit().isEmpty())
      {
        throw new DoubleJointedArmConfigurationException("Arm lower hard limit is empty",
                                                         "Cannot create simulation.",
                                                         "withHardLimit(Angle,Angle)");
      }
      if (lowerConfig.getUpperHardLimit().isEmpty() || upperConfig.getUpperHardLimit().isEmpty())
      {
        throw new DoubleJointedArmConfigurationException("Arm upper hard limit is empty",
                                                         "Cannot create simulation.",
                                                         "withHardLimit(Angle,Angle)");
      }

      // Setup Sim
      m_lowerArmSim = Optional.of(new SingleJointedArmSim(m_lowerSMC.getDCMotor(),
                                                          m_lowerSMC.getConfig().getGearing()
                                                                    .getMechanismToRotorRatio(),
                                                          lowerConfig.getMOI(),
                                                          lowerConfig.getLength().get().in(Meters),
                                                          lowerConfig.getLowerHardLimit().get().in(Radians),
                                                          lowerConfig.getUpperHardLimit().get().in(Radians),
                                                          true,
                                                          lowerConfig.getStartingAngle().get().in(Radians),
                                                          0.002 / 4096.0,
                                                          0.0));// Add noise with a std-dev of 1 tick
      m_lowerSMC.setSimSupplier(new ArmSimSupplier(m_lowerArmSim.get(), m_lowerSMC));
      m_upperArmSim = Optional.of(new SingleJointedArmSim(m_upperSMC.getDCMotor(),
                                                          m_upperSMC.getConfig().getGearing()
                                                                    .getMechanismToRotorRatio(),
                                                          upperConfig.getMOI(),
                                                          m_upperArmLength.in(Meters),
                                                          upperConfig.getLowerHardLimit().get().in(Radians),
                                                          upperConfig.getUpperHardLimit().get().in(Radians),
                                                          true,
                                                          upperConfig.getStartingAngle().get().in(Radians),
                                                          0.002 / 4096.0,
                                                          0.0));// Add noise with a std-dev of 1 tick
      m_upperSMC.setSimSupplier(new ArmSimSupplier(m_upperArmSim.get(), m_upperSMC));

      var lowerStartingAngle = lowerConfig.getStartingAngle().get();
      var upperStartingAngle = upperConfig.getStartingAngle().get();

      var upperArmRootPos = getJoint(m_lowerArmLength, lowerStartingAngle, m_lowerArmRootPos);

      var windowX =
          lowerMechPosCfg.getWindowXDimension(m_lowerArmLength).in(Meters) + upperMechPosCfg.getWindowXDimension(
              m_upperArmLength).in(Meters);
      var windowY =
          lowerMechPosCfg.getWindowYDimension(m_lowerArmLength).in(Meters) + upperMechPosCfg.getWindowYDimension(
              m_upperArmLength).in(Meters);

      m_mechanismWindow = new Mechanism2d(windowX, windowY);
      m_mechanismRoot = m_mechanismWindow.getRoot("Lower Root", m_lowerArmRootPos.getX(), m_lowerArmRootPos.getY());
      m_lowerLigament = m_mechanismLigament = m_mechanismRoot.append(
          new MechanismLigament2d(" lower", m_lowerArmLength.in(Meters), lowerStartingAngle.in(Degrees),
                                  7, lowerConfig.getSimColor()));
      m_upperRoot = m_mechanismWindow.getRoot("Upper Root", upperArmRootPos.getX(), upperArmRootPos.getY());
      m_upperLigament = m_upperRoot.append(new MechanismLigament2d("upper",
                                                                   m_upperArmLength.in(Meters),
                                                                   upperStartingAngle.in(Degrees),
                                                                   6,
                                                                   upperConfig.getSimColor()));
//      m_mechanismRoot.append(new MechanismLigament2d("MaxHard",
//                                                     Inch.of(3).in(Meters),
//                                                     lowerConfig.getUpperHardLimit().get()
//                                                         .in(Degrees),
//                                                     4,
//                                                     new Color8Bit(Color.kLimeGreen)));
//      m_mechanismRoot.append(new MechanismLigament2d("MinHard", Inch.of(3).in(Meters),
//                                                     lowerConfig.getLowerHardLimit().get()
//                                                         .in(Degrees),
//                                                     4, new Color8Bit(Color.kRed)));
//      if (m_lowerSMC.getConfig().getMechanismLowerLimit().isPresent() &&
//          m_lowerSMC.getConfig().getMechanismUpperLimit().isPresent())
//      {
//        m_mechanismRoot.append(new MechanismLigament2d("MaxSoft",
//                                                       Inch.of(3).in(Meters),
//                                                       m_lowerSMC.getConfig().getMechanismUpperLimit().get()
//                                                          .in(Degrees),
//                                                       4,
//                                                       new Color8Bit(Color.kHotPink)));
//        m_mechanismRoot.append(new MechanismLigament2d("MinSoft", Inch.of(3).in(Meters),
//                                                       m_lowerSMC.getConfig().getMechanismLowerLimit().get()
//                                                          .in(Degrees),
//                                                       4, new Color8Bit(Color.kYellow)));
//      }
      SmartDashboard.putData(getName() + "/mechanism", m_mechanismWindow);

      m_upperSMC.setupSimulation();
      m_lowerSMC.setupSimulation();
    }

    // Apply configs
    lowerConfig.applyConfig();
    upperConfig.applyConfig();
  }

  /**
   * Get the joint {@link Translation2d} of the arm in Meters.
   *
   * @param lowerArmLen   {@link Distance} length of the lower arm.
   * @param lowerArmAngle {@link Angle} angle of the lower arm.
   * @param lowerArmRoot  {@link Translation2d} root position of the lower arm.
   * @return {@link Translation2d} of the joint.
   */
  private Translation2d getJoint(Distance lowerArmLen, Angle lowerArmAngle, Translation2d lowerArmRoot)
  {
    return new Translation2d(lowerArmLen.times(Math.cos(lowerArmAngle.in(Radians))).in(Meters),
                             lowerArmLen.times(Math.sin(lowerArmAngle.in(Radians))).in(Meters)).plus(lowerArmRoot);
  }

  @Override
  public void updateTelemetry()
  {
//    m_telemetry.updatePosition(getAngle());
//    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint -> m_telemetry.updateSetpoint(m_setpoint));
    m_lowerSMC.updateTelemetry();
  }

  @Override
  public void simIterate()
  {
    if (m_lowerArmSim.isPresent() && m_lowerSMC.getSimSupplier().isPresent() && m_upperArmSim.isPresent() &&
        m_upperSMC.getSimSupplier().isPresent())
    {
      m_lowerSMC.getSimSupplier().get().updateSimState();
      m_lowerSMC.simIterate();
      m_lowerSMC.getSimSupplier().get().starveUpdateSim();
      m_upperSMC.getSimSupplier().get().updateSimState();
      m_upperSMC.simIterate();
      m_upperSMC.getSimSupplier().get().starveUpdateSim();
//      if (m_lowerArmConfig.getLowerHardLimit().isPresent() && m_lowerArmSim.get().getVelocityRadPerSec() < 0 &&
//          m_lowerSMC.getMechanismPosition().lt(m_lowerArmConfig.getLowerHardLimit().get()))
//      {
//        m_lowerSMC.setEncoderPosition(m_lowerArmConfig.getLowerHardLimit().get());
//      }
//      if (m_lowerArmConfig.getUpperHardLimit().isPresent() && m_lowerArmSim.get().getVelocityRadPerSec() > 0 &&
//          m_lowerSMC.getMechanismPosition().gt(m_lowerArmConfig.getUpperHardLimit().get()))
//      {
//        m_lowerSMC.setEncoderPosition(m_lowerArmConfig.getUpperHardLimit().get());
//      }
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_lowerArmSim.get()
                                                                                            .getCurrentDrawAmps(),
                                                                               m_upperArmSim.get()
                                                                                            .getCurrentDrawAmps()));
      visualizationUpdate();
    }
  }

  /**
   * Get the joint pose as a {@link Translation2d}.
   *
   * @return {@link Translation2d} of the joint.
   */
  private Translation2d getJointPose()
  {
    return getJoint(m_lowerArmLength, getLowerAngle(), m_lowerArmRootPos);
  }

  /**
   * Upper arm {@link Angle}
   *
   * @return {@link Angle} of the upper Arm.
   */
  public Angle getUpperAngle()
  {
    return m_upperSMC.getMechanismPosition();
  }

  /**
   * Lower arm {@link Angle}
   *
   * @return {@link Angle} of the lower arm.
   */
  public Angle getLowerAngle()
  {
    return m_lowerSMC.getMechanismPosition();
  }

  /**
   * Updates the mechanism ligament with the current angle of the arm.
   *
   * @see SmartPositionalMechanism#visualizationUpdate()
   */
  @Override
  public void visualizationUpdate()
  {
    var jointPos = getJointPose();
    m_lowerLigament.setAngle(getLowerAngle().in(Degrees));
    m_upperLigament.setAngle(getUpperAngle().in(Degrees));
    m_upperRoot.setPosition(jointPos.getX(), jointPos.getY());
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined in the
   * {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition()
  {
    Plane movementPlane = m_lowerArmConfig.getMechanismPositionConfig().getMovementPlane();
    Translation3d mechanismTranslation = new Translation3d(m_mechanismLigament.getLength(),
                                                           new Rotation3d(
                                                               Plane.YZ == movementPlane
                                                               ? m_mechanismLigament.getAngle()
                                                               : 0,
                                                               Plane.XZ == movementPlane
                                                               ? m_mechanismLigament.getAngle()
                                                               : 0, 0));
    if (m_lowerArmConfig.getMechanismPositionConfig().getRelativePosition().isPresent())
    {
      return m_lowerArmConfig.getMechanismPositionConfig().getRelativePosition().get()
                             .plus(mechanismTranslation);
    }
    return mechanismTranslation;
  }

  @Override
  public String getName()
  {
    return "DoubleJointedArm_" + m_lowerArmConfig.getTelemetryName().orElse("Lower") + "_" +
           m_upperArmConfig.getTelemetryName().orElse("Upper");
  }

  /**
   * Get the end pose of the arm.
   *
   * @return {@link Translation2d} of the arm.
   */
  private Translation2d getPosition()
  {
      return getJoint(m_upperArmLength, getUpperAngle(), getJoint(m_lowerArmLength, getLowerAngle(), m_lowerArmRootPos));
  }

  public void setPosition()
  {

  }

  /**
   * Arm is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return Trigger on when the arm is near another angle.
   */
  public Trigger isNear(Angle angle, Angle within)
  {
    return null;
//    return new Trigger(() -> getAngle().isNear(angle, within));
  }

  @Override
  public Trigger max()
  {
    if (m_lowerSMC.getConfig().getMechanismUpperLimit().isPresent())
    {
      return new Trigger(gte(m_lowerSMC.getConfig().getMechanismUpperLimit().get()));
    }
    if (m_lowerArmConfig.getUpperHardLimit().isEmpty())
    {
      return gte(m_lowerArmConfig.getUpperHardLimit().get());
    }
    throw new ArmConfigurationException("Arm upper hard and motor controller soft limit is empty",
                                        "Cannot create max trigger.",
                                        "withHardLimit(Angle,Angle)");
  }

  @Override
  public Command set(double dutycycle)
  {
    return Commands.runOnce(() -> m_lowerSMC.setDutyCycle(dutycycle));
  }

  @Override
  public Command set(Supplier<Double> dutycyle)
  {
    return Commands.runOnce(() -> m_lowerSMC.setDutyCycle(dutycyle.get()));
  }

  @Override
  public Command setVoltage(Voltage volts)
  {
    return Commands.runOnce(() -> m_lowerSMC.setVoltage(volts));
  }

  public Command setVoltage(Supplier<Voltage> volts)
  {
    return Commands.runOnce(() -> m_lowerSMC.setVoltage(volts.get()));
  }


  @Override
  public Trigger min()
  {
    if (m_lowerSMC.getConfig().getMechanismLowerLimit().isPresent())
    {
      return new Trigger(gte(m_lowerSMC.getConfig().getMechanismLowerLimit().get()));
    }
    if (m_lowerArmConfig.getLowerHardLimit().isEmpty())
    {
      return gte(m_lowerArmConfig.getLowerHardLimit().get());
    }
    throw new ArmConfigurationException("Arm lower hard and motor controller soft limit is empty",
                                        "Cannot create min trigger.",
                                        "withHardLimit(Angle,Angle)");
  }

  @Override
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    SysIdRoutine routine = m_lowerSMC.sysId(maximumVoltage, step, duration);
    Angle        max;
    Angle        min;
    if (m_lowerSMC.getConfig().getMechanismUpperLimit().isPresent())
    {
      max = m_lowerSMC.getConfig().getMechanismUpperLimit().get().minus(Degrees.of(1));
    } else if (m_lowerArmConfig.getUpperHardLimit().isPresent())
    {
      max = m_lowerArmConfig.getUpperHardLimit().get().minus(Degrees.of(1));
    } else
    {
      throw new ArmConfigurationException("Arm upper hard and motor controller soft limit is empty",
                                          "Cannot create SysIdRoutine.",
                                          "withHardLimit(Angle,Angle)");
    }
    if (m_lowerSMC.getConfig().getMechanismLowerLimit().isPresent())
    {
      min = m_lowerSMC.getConfig().getMechanismLowerLimit().get().plus(Degrees.of(1));
    } else if (m_lowerArmConfig.getLowerHardLimit().isPresent())
    {
      min = m_lowerArmConfig.getLowerHardLimit().get().plus(Degrees.of(1));
    } else
    {
      throw new ArmConfigurationException("Arm lower hard and motor controller soft limit is empty",
                                          "Cannot create SysIdRoutine.",
                                          "withHardLimit(Angle,Angle)");
    }
    Trigger maxTrigger = gte(max);
    Trigger minTrigger = lte(min);

    Command group = Commands.print("Starting SysId!")
                            .andThen(Commands.runOnce(m_lowerSMC::stopClosedLoopController))
                            .andThen(routine.dynamic(Direction.kForward).until(maxTrigger))
                            .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
                            .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
                            .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger))
                            .finallyDo(m_lowerSMC::startClosedLoopController);
    if (m_lowerArmConfig.getTelemetryName().isPresent())
    {
      group = group.andThen(Commands.print(getName() + " SysId test done."));
    }
    return group.withName(m_subsystem.getName() + " SysId");
  }

  /**
   * Between two angles.
   *
   * @param start Start angle.
   * @param end   End angle
   * @return {@link Trigger}
   */
  public Trigger between(Angle start, Angle end)
  {
    return gte(start).and(lte(end));
  }

  /**
   * Less than or equal to angle
   *
   * @param angle {@link Angle} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Angle angle)
  {
    return null;
//    return new Trigger(() -> getAngle().lte(angle));
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return {@link Trigger} for Arm.
   */
  public Trigger gte(Angle angle)
  {
    return null;
//    return new Trigger(() -> getAngle().gte(angle));
  }

  /**
   * Get the {@link ArmConfig} for this {@link DoubleJointedArm}.
   *
   * @return The {@link ArmConfig} used to configure this {@link DoubleJointedArm}.
   */
  public ArmConfig getArmConfig()
  {
    return m_lowerArmConfig;
  }
}
