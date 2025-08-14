package yams.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import java.util.Map;

import static edu.wpi.first.units.Units.*;

/**
 * Smart motor controller telemetry.
 */
public class SmartMotorControllerTelemetry
{


  /**
   * {@link TelemetryVerbosity} for the {@link SmartMotorController}
   */
  private final TelemetryVerbosity                           verbosity = TelemetryVerbosity.HIGH;
  /**
   * Double telemetry fields that will be outputted.
   */
  private       Map<DoubleTelemetryField, DoubleTelemetry>   doubleFields;
  /**
   * Boolean telemetry fields that will be outputted.
   */
  private       Map<BooleanTelemetryField, BooleanTelemetry> boolFields;

  /**
   * Network table to publish to.
   */
  private NetworkTable                        dataNetworkTable;
  /**
   * Network table for tuning.
   */
  private NetworkTable                        tuningNetworkTable;
  /**
   * Telemetry config
   */
  private SmartMotorControllerTelemetryConfig config;

  /**
   * Setup Telemetry Pub/Sub fields.
   *
   * @param smartMotorController {@link SmartMotorController} used to determine what telemetry features can and should
   *                             be disabled bc they are not available.
   * @param publishTable         {@link NetworkTable} that holds all of the output fields.
   * @param tuningTable          {@link NetworkTable} that holds all of the tuning fields.
   * @param config               {@link SmartMotorControllerTelemetryConfig} to apply.
   */
  public void setupTelemetry(SmartMotorController smartMotorController, NetworkTable publishTable,
                             NetworkTable tuningTable,
                             SmartMotorControllerTelemetryConfig config)
  {
    if (!publishTable.equals(this.dataNetworkTable))
    {
      dataNetworkTable = publishTable;
      tuningNetworkTable = tuningTable;
      this.config = config;
      doubleFields = config.getDoubleFields(smartMotorController);
      boolFields = config.getBoolFields(smartMotorController);
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }

    }
  }


  /**
   * Publish {@link SmartMotorController} telemetry to {@link NetworkTable}
   *
   * @param smc Smart motor controller to publish telemetry for.
   */
  public void publish(SmartMotorController smc)
  {
    SmartMotorControllerConfig cfg = smc.getConfig();
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry> entry : boolFields.entrySet())
    {
      BooleanTelemetry bt = entry.getValue();
      if (!bt.enabled)
      {
        continue;
      }
      switch (bt.getField())
      {
        case MechanismUpperLimit ->
        {
          cfg.getMechanismUpperLimit().ifPresent(upperLimit -> bt.set(smc.getMechanismPosition().gte(upperLimit)));
        }
        case MechanismLowerLimit ->
        {
          cfg.getMechanismLowerLimit().ifPresent(lowerLimit -> bt.set(smc.getMechanismPosition().lte(lowerLimit)));
        }
        case TemperatureLimit ->
        {
          cfg.getTemperatureCutoff().ifPresent(temperatureCutoff -> bt.set(smc.getTemperature()
                                                                              .gte(temperatureCutoff)));
        }
        case VelocityControl ->
        {
          bt.set(smc.getMechanismSetpointVelocity().isPresent());
        }
        case ElevatorFeedForward ->
        {
          bt.set(cfg.getElevatorFeedforward().isPresent());
        }
        case ArmFeedForward ->
        {
          bt.set(cfg.getArmFeedforward().isPresent());
        }
        case SimpleMotorFeedForward ->
        {
          bt.set(cfg.getSimpleFeedforward().isPresent());
        }
        case MotionProfile ->
        {
          bt.set(cfg.getClosedLoopController().isPresent());
        }
      }
    }
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
    {
      DoubleTelemetry dt = entry.getValue();
      if (!dt.enabled)
      {
        continue;
      }
      switch (dt.getField())
      {
        case SetpointPosition ->
        {
          smc.getMechanismPositionSetpoint().ifPresent(mechSetpoint -> dt.set(
              cfg.getMechanismCircumference().isPresent() ? cfg.convertFromMechanism(mechSetpoint).in(Meters)
                                                          : mechSetpoint.in(Rotations)));
          break;
        }
        case SetpointVelocity ->
        {
          smc.getMechanismSetpointVelocity().ifPresent(mechSetpoint ->
                                                           dt.set(cfg.getMechanismCircumference().isPresent() ?
                                                                  cfg.convertFromMechanism(mechSetpoint)
                                                                     .in(MetersPerSecond) :
                                                                  mechSetpoint.in(RotationsPerSecond)));
        }
        case OutputVoltage ->
        {
          dt.set(smc.getVoltage().in(Volts));
        }
        case StatorCurrent ->
        {
          dt.set(smc.getStatorCurrent().in(Amps));
        }
        case SupplyCurrent ->
        {
          smc.getSupplyCurrent().ifPresent(current -> dt.set(current.in(Amps)));
        }
        case MotorTemperature ->
        {
          dt.set(smc.getTemperature().in(Fahrenheit));
        }
        case MeasurementPosition ->
        {
          cfg.getMechanismCircumference().ifPresent(circumference -> dt.set(smc.getMeasurementPosition().in(Meters)));
        }
        case MeasurementVelocity ->
        {
          cfg.getMechanismCircumference().ifPresent(circumference -> dt.set(smc.getMeasurementVelocity()
                                                                               .in(MetersPerSecond)));
        }
        case MechanismPosition ->
        {
          dt.set(smc.getMechanismPosition().in(Rotations));
        }
        case MechanismVelocity ->
        {
          dt.set(smc.getMechanismVelocity().in(RotationsPerSecond));
        }
        case RotorPosition ->
        {
          dt.set(smc.getRotorPosition().in(Rotations));
        }
        case RotorVelocity ->
        {
          dt.set(smc.getRotorVelocity().in(RotationsPerSecond));
        }
      }
    }
  }


  /**
   * Apply the tuning values from {@link NetworkTable} to the {@link SmartMotorController}
   *
   * @param smartMotorController {@link SmartMotorController} to control.
   */
  public void applyTuningValues(SmartMotorController smartMotorController)
  {
    SmartMotorControllerConfig cfg = smartMotorController.getConfig();
    if(cfg.getMotorControllerMode() != SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
    {
      throw new SmartMotorControllerConfigurationException("Live tuning does not work in OPEN_LOOP", "Cannot apply setpoints for Live Tuning.",".withControlMode(ControlMode.CLOSED_LOOP) instead of .withControlMode(ControlMode.OPEN_LOOP)");
    }
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry> entry : boolFields.entrySet())
    {
      BooleanTelemetry bt = entry.getValue();
      if (bt.tunable())
      {
        switch (bt.getField())
        {
          case MotorInversion -> smartMotorController.setMotorInverted(bt.get());
          case EncoderInversion -> smartMotorController.setEncoderInverted(bt.get());
        }
      }
    }
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
    {
      DoubleTelemetry dt = entry.getValue();
      if (dt.tunable())
      {
        switch (dt.getField()) {
          case TunableSetpointPosition -> {
            System.out.println("SET ELEV POS TO: "+dt.get());
            cfg.getMechanismCircumference().ifPresentOrElse(
                    circumference -> {
                      smartMotorController.setPosition(Meters.of(dt.get()));
                    }, () -> {
                      smartMotorController.setPosition(Rotations.of(dt.get()));
                    });
            break;
          }
          case TunableSetpointVelocity -> {
            if (dt.get() == 0 && smartMotorController.getMechanismSetpointVelocity().isEmpty()) {
              continue;
            }
            cfg.getMechanismCircumference().ifPresentOrElse(circumference -> smartMotorController.setVelocity(
                            MetersPerSecond.of(
                                    dt.get())),
                    () -> smartMotorController.setVelocity(
                            dt.get() == 0 ? null : RadiansPerSecond.of(dt.get())));
            break;
          }
          case kP -> smartMotorController.setKp(dt.get());
          case kI -> smartMotorController.setKi(dt.get());
          case kD -> smartMotorController.setKd(dt.get());
          case kS -> smartMotorController.setKs(dt.get());
          case kV -> smartMotorController.setKv(dt.get());
          case kA -> smartMotorController.setKa(dt.get());
          case kG -> smartMotorController.setKg(dt.get());
          case ClosedloopRampRate -> smartMotorController.setClosedLoopRampRate(Seconds.of(dt.get()));
          case OpenloopRampRate -> smartMotorController.setOpenLoopRampRate(Seconds.of(dt.get()));
          case SupplyCurrentLimit -> smartMotorController.setSupplyCurrentLimit(Amps.of(dt.get()));
          case StatorCurrentLimit -> smartMotorController.setStatorCurrentLimit(Amps.of(dt.get()));
          case MeasurementUpperLimit -> smartMotorController.setMeasurementUpperLimit(Meters.of(dt.get()));
          case MeasurementLowerLimit -> smartMotorController.setMeasurementLowerLimit(Meters.of(dt.get()));
          case MechanismUpperLimit -> smartMotorController.setMechanismUpperLimit(Rotations.of(dt.get()));
          case MechanismLowerLimit -> smartMotorController.setMechanismLowerLimit(Rotations.of(dt.get()));
          case MotionProfileMaxAcceleration ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(distance -> {
                                                              smartMotorController.setMotionProfileMaxAcceleration(MetersPerSecondPerSecond.of(dt.get()));
                                                            },
                                                            () -> {
                                                              smartMotorController.setMotionProfileMaxAcceleration(
                                                                  RotationsPerSecondPerSecond.of(dt.get()));
                                                            });
            break;
          }
          case MotionProfileMaxVelocity ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(distance -> {
                                                              smartMotorController.setMotionProfileMaxVelocity(MetersPerSecond.of(dt.get()));
                                                            },
                                                            () -> {
                                                              smartMotorController.setMotionProfileMaxVelocity(
                                                                  RotationsPerSecond.of(dt.get()));
                                                            });

            break;
          }
        }
      }
    }
  }

  /**
   * Boolean telemetry for {@link SmartMotorController}s
   */
  public enum BooleanTelemetryField
  {
    /**
     * Mechanism upper limit
     */
    MechanismUpperLimit("limits/Mechanism Upper Limit", false, false),
    /**
     * Mechanism lower limit.
     */
    MechanismLowerLimit("limits/Mechanism Lower Limit", false, false),
    /**
     * Temperature limit if available.
     */
    TemperatureLimit("limits/Temperature Limit", false, false),
    /**
     * Velocity control currently getting used.
     */
    VelocityControl("control/Velocity Control", false, false),
    /**
     * Elevator feedforward currently getting used.
     */
    ElevatorFeedForward("control/Elevator Feedforward", false, false),
    /**
     * Arm feedforward currently getting used.
     */
    ArmFeedForward("control/Arm Feedforward", false, false),
    /**
     * Simple motor feedforward currently getting used.
     */
    SimpleMotorFeedForward("control/Simple Motor Feedforward", false, false),
    /**
     * Motion profile currently getting used.
     */
    MotionProfile("control/Motion Profile", false, false),
    /**
     * Motor inversion.
     */
    MotorInversion("motor/inverted", false, true),
    /**
     * Encoder inversion.
     */
    EncoderInversion("encoder/inverted", false, true);

    /**
     * Default value of the boolean telemetry field.
     */
    private final boolean currentValue;
    /**
     * Key that the telemetry is stored at.
     */
    private final String  key;
    /**
     * Tunable field?
     */
    private final boolean tunable;

    /**
     * Create a boolean telemetry field.
     *
     * @param fieldName    Field for {@link NetworkTable}
     * @param defaultValue Default value in NT.
     * @param tunable      Tunable field.
     */
    BooleanTelemetryField(String fieldName, boolean defaultValue, boolean tunable)
    {
      key = fieldName;
      currentValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link BooleanTelemetry} object for non-static usage.
     *
     * @return {@link BooleanTelemetry}
     */
    public BooleanTelemetry create()
    {
      return new BooleanTelemetry(key, currentValue, this, tunable);
    }


  }

  /**
   * Double telemetry field.
   */
  public enum DoubleTelemetryField
  {
    /**
     * Motion profile maximum velocity, could be in MPS or RPS
     */
    MotionProfileMaxVelocity("closedloop/motionprofile/maxVelocity", 0, true),
    /**
     * Motion profile maximum accelerartion, could be in MPS^2 or RPS^2
     */
    MotionProfileMaxAcceleration("closedloop/motionprofile/maxAcceleration", 0, true),
    /**
     * kS
     */
    kS("closedloop/feedforward/kS", 0, true),
    /**
     * kV
     */
    kV("closedloop/feedforward/kV", 0, true),
    /**
     * kA
     */
    kA("closedloop/feedforward/kA", 0, true),
    /**
     * kG
     */
    kG("closedloop/feedforward/kG", 0, true),
    /**
     * kP
     */
    kP("closedloop/feedback/kP", 0, true),
    /**
     * kI
     */
    kI("closedloop/feedback/kI", 0, true),
    /**
     * kD
     */
    kD("closedloop/feedback/kD", 0, true),
    /**
     * Setpoint position
     */
    TunableSetpointPosition("closedloop/setpoint/position", 0, true),
    /**
     * Setpoint position
     */
    SetpointPosition("closedloop/setpoint/position", 0, false),
    /**
     * Setpoint velocity
     */
    TunableSetpointVelocity("closedloop/setpoint/velocity", 0, true),
    /**
     * Setpoint velocity
     */
    SetpointVelocity("closedloop/setpoint/velocity", 0, false),
    /**
     * Voltage supplied to the motor.
     */
    OutputVoltage("motor/outputVoltage", 0, false),
    /**
     * Stator current.
     */
    StatorCurrent("current/stator", 0, false),
    /**
     * Supply current limit.
     */
    StatorCurrentLimit("current/limit/stator", 0, true),
    /**
     * Supply current.
     */
    SupplyCurrent("current/supply", 0, false),
    /**
     * Supply current limit.
     */
    SupplyCurrentLimit("current/limit/supply", 0, true),
    /**
     * Motor temperature
     */
    MotorTemperature("motor/temperature", 0, false),
    /**
     * Measurement position
     */
    MeasurementPosition("measurement/position (meters)", 0, false),
    /**
     * Measurement velocity.
     */
    MeasurementVelocity("measurement/velocity (meters per second)", 0, false),
    /**
     * Measurement lower limit.
     */
    MeasurementLowerLimit("measurement/limit/lower (meters)", 0, true),
    /**
     * Measurement upper limit.
     */
    MeasurementUpperLimit("measurement/limit/upper (meters)", 0, true),
    /**
     * Mechanism position in rotations.
     */
    MechanismPosition("mechanism/position (rotations)", 0, false),
    /**
     * Mechanism velocity in rotations per second.
     */
    MechanismVelocity("mechanism/velocity (rotations per second)", 0, false),
    /**
     * Mechanism lower limit in rotations.
     */
    MechanismLowerLimit("mechanism/limit/lower (rotations)", 0, true),
    /**
     * Mechanism upper limit in rotations.
     */
    MechanismUpperLimit("mechanism/limit/upper (rotations)", 0, true),
    /**
     * Rotor position in rotations.
     */
    RotorPosition("rotor/position (rotations)", 0, false),
    /**
     * Rotor velocity in rotations.
     */
    RotorVelocity("rotor/velocity (rotations per second)", 0, false),
    /**
     * Closed loop dutycyle ramp rate.
     */
    ClosedloopRampRate("ramprate/dutycyle/closedloop", 0, true),
    /**
     * Open loop dutycycle ramp rate.
     */
    OpenloopRampRate("ramprate/dutycycle/openloop", 0, true);

    private final double  defaultVal;
    private final String  key;
    private final boolean tunable;

    DoubleTelemetryField(String fieldName, double defaultValue, boolean tunable)
    {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create double telemetry field.
     *
     * @return Double telemetry field.
     */
    public DoubleTelemetry create()
    {
      return new DoubleTelemetry(key, defaultVal, this, tunable);
    }

  }
}
