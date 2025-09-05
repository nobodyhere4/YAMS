// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.telemetry;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSub;
import java.util.Optional;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * Add your docs here.
 */
public class DoubleTelemetry
{

  /**
   * Field representing.
   */
  private final DoubleTelemetryField       field;
  /**
   * Network table key.
   */
  private final String                     key;
  /**
   * Tunable?
   */
  private final boolean                    tunable;
  /**
   * Unit to display.
   */
  private       String                     unit;
  /**
   * Enabled?
   */
  protected     boolean                    enabled      = false;
  /**
   * Default value.
   */
  private       double                     defaultValue;
  /**
   * Cached value.
   */
  private       double                     cachedValue;
  /**
   * Publisher.
   */
  private       DoublePublisher            publisher    = null;
  /**
   * Subscriber.
   */
  private       Optional<DoubleSubscriber> subscriber   = Optional.empty();
  /**
   * Sub publisher.
   */
  private       DoublePublisher            subPublisher = null;
  /**
   * Tuning table
   */
  private       Optional<NetworkTable>     tuningTable  = Optional.empty();
  /**
   * Data table.
   */
  private       Optional<NetworkTable>     dataTable    = Optional.empty();
  private       DoubleTopic                topic;


  /**
   * Setup double telemetry for a field.
   *
   * @param keyString  Key to use.
   * @param defaultVal Default value.
   * @param field      Field representing.
   * @param tunable    Tunable.
   * @param unit       Unit to display.
   */
  public DoubleTelemetry(String keyString, double defaultVal, DoubleTelemetryField field, boolean tunable, String unit)
  {
    key = keyString;
    cachedValue = defaultValue = defaultVal;
    this.field = field;
    this.tunable = tunable;
    this.unit = unit;
  }

  /**
   * Set default values.
   *
   * @param defaultValue Default for the entry.
   */
  public void setDefaultValue(double defaultValue)
  {
    cachedValue = this.defaultValue = defaultValue;
  }

  /**
   * Setup network tables.
   *
   * @param dataTable   Data tables.
   * @param tuningTable Tuning table.
   */
  public void setupNetworkTables(NetworkTable dataTable, NetworkTable tuningTable)
  {
    this.tuningTable = Optional.ofNullable(tuningTable);
    this.dataTable = Optional.ofNullable(dataTable);
    if (tuningTable != null && tunable)
    {
      topic = tuningTable.getDoubleTopic(key);
      subscriber = Optional.of(topic.subscribe(defaultValue));
      subPublisher = topic.publish();
      if (!unit.equals("none"))
      {topic.setProperties("{\"unit\":\""+ unit+"\"}");}
      subPublisher.setDefault(defaultValue);
    } else
    {
      assert dataTable != null;
      topic = dataTable.getDoubleTopic(key);
      publisher = topic.publish();
      if (!unit.equals("none"))
      {topic.setProperties("{\"unit\": \""+ unit+"\"}");}
      publisher.setDefault(defaultValue);
    }
  }

  /**
   * Set the unit.
   *
   * @return {@link DoubleTelemetry} for chaining.
   */
  public DoubleTelemetry transformUnit(SmartMotorControllerConfig cfg)
  {
    switch (unit)
    {
      case "position":
        unit = cfg.getMechanismCircumference().isPresent() ? "meters" : "rotations";
        break;
      case "velocity":
        unit = cfg.getMechanismCircumference().isPresent() ? "meters_per_second" : "rotations_per_second";
        break;
      case "acceleration":
        unit = cfg.getMechanismCircumference().isPresent() ? "meters_per_second_per_second"
                                                           : "rotations_per_second_per_second";
        break;
    }
    return this;
  }


  /**
   * Setup network tables.
   *
   * @param dataTable Data tables.
   */
  public void setupNetworkTable(NetworkTable dataTable)
  {
    setupNetworkTables(dataTable, null);
  }

  /**
   * Set the value of the publisher, checking to see if the value is the same as the subscriber.
   *
   * @param value Value to set.
   * @return True if value was able to be set.
   */
  public boolean set(double value)
  {
    if (subscriber.isPresent())
    {
      double tuningValue = subscriber.get().get(defaultValue);
      if (tuningValue != value)
      {
        return false;
      }
    }
    if (publisher != null)
    {
      publisher.accept(value);
    }
    return true;
  }

  /**
   * Get the value.
   *
   * @return value of telemetry.
   */
  public double get()
  {
    if (subscriber.isPresent())
    {
      return subscriber.get().get(defaultValue);
    }
    throw new RuntimeException("Tuning table not configured for " + key + "!");
  }

  /**
   * Check to see if the value has changed.
   *
   * @return True if the value has changed.
   */
  public boolean tunable()
  {
    if (subscriber.isPresent() && tunable && enabled)
    {
      if (subscriber.get().get(defaultValue) != cachedValue)
      {
        cachedValue = subscriber.get().get(defaultValue);
        return true;
      }
      return false;
    }
    return false;
  }

  /**
   * Enable the telemetry.
   */
  public void enable()
  {
    enabled = true;
  }

  /**
   * Disable the telemetry.
   */
  public void disable()
  {
    enabled = false;
  }

  /**
   * Display the telemetry.
   *
   * @param state Enable or disable.
   */
  public void display(boolean state)
  {
    enabled = state;
  }

  /**
   * Get the field.
   *
   * @return field.
   */
  public DoubleTelemetryField getField()
  {
    return field;
  }

  /**
   * Close the telemetry field.
   */
  public void close()
  {
    subscriber.ifPresent(PubSub::close);
    if (subPublisher != null)
    {subPublisher.close();}
    if (publisher != null)
    {publisher.close();}
    dataTable.ifPresent(table -> table.getEntry(key).unpublish());
    tuningTable.ifPresent(table -> table.getEntry(key).unpublish());
  }
}
