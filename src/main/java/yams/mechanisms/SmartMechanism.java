package yams.mechanisms;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.gearing.gearbox.GearBox;
import yams.gearing.gearbox.GearBox.Type;
import yams.gearing.gearbox.MAXPlanetaryGearbox;
import yams.gearing.gearbox.VersaPlanetaryGearBox;
import yams.motorcontrollers.SmartMotorController;

/**
 * Generic implementation of a mechanism with advanced telemetry.
 */
public abstract class SmartMechanism
{

  /**
   * SysId routine to tune the mechanism.
   */
  protected Optional<SysIdRoutine>          m_sysIdRoutine  = Optional.empty();
  /**
   * Motor controller for the primary motor in the mechanism.
   */
  protected Optional<SmartMotorController>  m_motor         = Optional.empty();
  /**
   * {@link MechanismGearing} for the mechanism.
   */
  protected Optional<MechanismGearing>      m_gearing       = Optional.empty();
  /**
   * Name of the mechanism for command decoration and telemetry.
   */
  protected Optional<String>                m_name          = Optional.empty();
  /**
   * PID controller for the mechanism.
   */
  protected Optional<ProfiledPIDController> m_pidController = Optional.empty();
  /**
   * Mechanism {@link NetworkTable}
   */
  protected NetworkTable                    m_networkTable  = NetworkTableInstance.getDefault().getTable(
      "SmartDashboard");
  /**
   * Boolean publisher that is true when the {@link SmartMotorController} is experiencing problems or does not exist.
   */
  protected BooleanPublisher                m_motorError;

  /**
   * Create the {@link Sprocket} class easily for use within the mechanism.
   *
   * @param sprockets Teeth of each sprocket in the chain, must be at 2 sprocket teeth defined.
   * @return {@link Sprocket} representing the sprockets given.
   */
  protected Sprocket sprocket(double... sprockets)
  {
    return new Sprocket(sprockets);
  }

  /**
   * Create the {@link GearBox} for {@link MechanismGearing}
   *
   * @param type   {@link GearBox.Type} to create.
   * @param stages Stages in the gear box.
   * @return {@link GearBox} for use in {@link MechanismGearing};
   */
  protected GearBox gearbox(GearBox.Type type, double... stages)
  {
    switch (type)
    {
      case MAX_PLANETARY ->
      {
        return new MAXPlanetaryGearbox(stages);
      }
      case VERSA_PLANETARY ->
      {
        return new VersaPlanetaryGearBox(stages);
      }
    }
    throw new IllegalArgumentException("Unknown GearBox type: " + type);
  }

  /**
   * Create and set the {@link SmartMechanism#m_gearing} attribute with the given {@link GearBox} and {@link Sprocket}
   *
   * @param gearBox  {@link GearBox} created using {@link SmartMechanism#gearbox(Type, double...)}.
   * @param sprocket {@link Sprocket} created using {@link SmartMechanism#sprocket(double...)}.
   * @return {@link MechanismGearing} with the {@link GearBox} and {@link Sprocket}.
   */
  protected MechanismGearing gearing(GearBox gearBox, Sprocket sprocket)
  {
    m_gearing = Optional.of(new MechanismGearing(gearBox, sprocket));
    return m_gearing.get();
  }

  /**
   * Create and set the {@link SmartMechanism#m_gearing} attribute with the given {@link GearBox}.
   *
   * @param gearBox {@link GearBox} created using {@link SmartMechanism#gearbox(Type, double...)}.
   * @return {@link MechanismGearing} with the {@link GearBox}.
   */
  protected MechanismGearing gearing(GearBox gearBox)
  {
    m_gearing = Optional.of(new MechanismGearing(gearBox));
    return m_gearing.get();
  }
}
