package yams.telemetry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import yams.motorcontrollers.SmartMotorController;

public class MechanismTelemetry
{

  /**
   * Telemetry NetworkTable.
   */
  private NetworkTable         networkTable;
  /**
   * Tuning NetworkTable.
   */
  private NetworkTable         tuningNetworkTable;
  /**
   * Network Tables publisher for the units displayed by the positional mechanism.
   */
  private StringPublisher      units;
  /**
   * Units for the mechanism setpoint and position.
   */
  private String               unitsString;
  /**
   * Network Tables Publisher for the setpoint.
   */
  private DoublePublisher      setpointPublisher;
  /**
   * Network Tables publisher for the position of the mechanism.
   */
  private DoublePublisher      positionPublisher;
  private SmartMotorController motorController;

  /**
   * Setup telemetry for the Mechanism
   *
   * @param mechanismTelemetryName Mechanism Telemetry Name.
   * @param motorController        {@link SmartMotorController} to setup telemetry for.
   * @param units                  Units of the Mechanism.
   * @param position               Position of the mechanism.
   * @param setpoint               Setpoint of the mechanism.
   */
  public void setupTelemetry(String mechanismTelemetryName, SmartMotorController motorController, String units,
                             double setpoint, double position)
  {
    tuningNetworkTable = NetworkTableInstance.getDefault().getTable("Tuning")
                                             .getSubTable(mechanismTelemetryName);
    networkTable = NetworkTableInstance.getDefault().getTable("Mechanisms")
                                       .getSubTable(mechanismTelemetryName);
    this.motorController = motorController;
    this.unitsString = units;
    this.units = tuningNetworkTable.getStringTopic("Units").publish();
    setpointPublisher = tuningNetworkTable.getDoubleTopic("Setpoint").publish();
    positionPublisher = networkTable.getDoubleTopic("Position").publish();
    this.units.set(units);
    this.setpointPublisher.set(setpoint);
    this.positionPublisher.set(position);
  }

  /**
   * Update the position of the mechanism using the Angle.
   *
   * @param position Angular position of the mechanism.
   */
  public void updatePosition(Angle position)
  {
    switch (unitsString)
    {
      case "Degrees":
        positionPublisher.set(position.in(Degrees));
        break;
      case "Radians":
        positionPublisher.set(position.in(Radians));
        break;
      default:
        throw new IllegalArgumentException(
            positionPublisher.getTopic().toString() + ": Invalid units given to mechanism telemetry!");
    }
    // TODO: AKit logging here.

  }

  /**
   * Update the position for distance based mechanisms.
   *
   * @param position Distance of the mechanism.
   */
  public void updatePosition(Distance position)
  {
    switch (unitsString)
    {
      case "Meters":
        positionPublisher.set(position.in(Meters));
        break;
      case "Feet":
        positionPublisher.set(position.in(Feet));
        break;
      default:
        throw new IllegalArgumentException(
            positionPublisher.getTopic().toString() + ": Invalid units given to mechanism telemetry!");
    }
    // TODO: AKit logging here.

  }


  /**
   * Update the setpoint telemetry for the mechanism.
   *
   * @param setpoint Setpoint of the Mechanism in Distance.
   */
  public void updateSetpoint(Distance setpoint)
  {
    switch (unitsString)
    {
      case "Meters":
        positionPublisher.set(setpoint.in(Meters));
        break;
      case "Feet":
        positionPublisher.set(setpoint.in(Feet));
        break;
      default:
        throw new IllegalArgumentException(
            positionPublisher.getTopic().toString() + ": Invalid units given to mechanism telemetry!");
    }
  }

  /**
   * Update the setpoint as an Angle.
   *
   * @param setpoint Setpoint of the mechanism as an Angle.
   */
  public void updateSetpoint(Angle setpoint)
  {
    switch (unitsString)
    {
      case "Degrees":
        positionPublisher.set(setpoint.in(Degrees));
        break;
      case "Radians":
        positionPublisher.set(setpoint.in(Radians));
        break;
      default:
        throw new IllegalArgumentException(
            positionPublisher.getTopic().toString() + ": Invalid units given to mechanism telemetry!");
    }
  }


  public void updateUnits(String units)
  {
    if (!List.of("Meters", "Feet", "Degrees", "Radians").contains(units))
    {
      throw new IllegalArgumentException("Invalid units given to mechanism telemetry!");
    }
    this.units.set(units);
  }
}
