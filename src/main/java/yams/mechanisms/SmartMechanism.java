package yams.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.gearing.gearbox.GearBox;
import yams.gearing.gearbox.GearBox.Type;
import yams.gearing.gearbox.MAXPlanetaryGearbox;
import yams.gearing.gearbox.VersaPlanetaryGearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.telemetry.MechanismTelemetry;

/**
 * Generic implementation of a mechanism with advanced telemetry.
 */
public abstract class SmartMechanism
{

  /**
   * Subsystem for the Mechanism.
   */
  protected Subsystem            m_subsystem;
  /**
   * Motor for the subsystem.
   */
  protected SmartMotorController m_motor;
  /**
   * Mechanism telemetry.
   */
  protected MechanismTelemetry m_telemetry = new MechanismTelemetry();
  /**
   * Mechanism Window.
   */
  protected Mechanism2d        mechanismWindow;

  /**
   * Create the {@link Sprocket} class easily for use within the mechanism.
   *
   * @param sprocketReductionStages Teeth of each sprocket in the chain.
   * @return {@link Sprocket} representing the sprocketReductionStages given.
   */
  public static Sprocket sprocket(double... sprocketReductionStages)
  {
    return new Sprocket(sprocketReductionStages);
  }

  /**
   * Create the {@link GearBox} for {@link MechanismGearing}
   *
   * @param type            {@link GearBox.Type} to create.
   * @param reductionStages Reduction stages in the gear box.
   * @return {@link GearBox} for use in {@link MechanismGearing};
   */
  public static GearBox gearbox(GearBox.Type type, double... reductionStages)
  {
    switch (type)
    {
      case MAX_PLANETARY ->
      {
        return new MAXPlanetaryGearbox(reductionStages);
      }
      case VERSA_PLANETARY ->
      {
        return new VersaPlanetaryGearBox(reductionStages);
      }
    }
    throw new IllegalArgumentException("Unknown GearBox type: " + type);
  }

  /**
   * Create {@link MechanismGearing} with the given {@link GearBox} and {@link Sprocket}
   *
   * @param gearBox  {@link GearBox} created using {@link SmartMechanism#gearbox(Type, double...)}.
   * @param sprocket {@link Sprocket} created using {@link SmartMechanism#sprocket(double...)}.
   * @return {@link MechanismGearing} with the {@link GearBox} and {@link Sprocket}.
   */
  public static MechanismGearing gearing(GearBox gearBox, Sprocket sprocket)
  {
    return new MechanismGearing(gearBox, sprocket);
  }

  /**
   * Create {@link MechanismGearing} with the given {@link GearBox}.
   *
   * @param gearBox {@link GearBox} created using {@link SmartMechanism#gearbox(Type, double...)}.
   * @return {@link MechanismGearing} with the {@link GearBox}.
   */
  public static MechanismGearing gearing(GearBox gearBox)
  {
    return new MechanismGearing(gearBox);
  }
}
