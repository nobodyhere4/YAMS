// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.telemetry;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import java.util.Optional;

/**
 * Add your docs here.
 */
public class DoubleTelemetry
{

  private final String                     key;
  private final double                     defaultValue;
  private       boolean                    enabled    = false;
  private       DoublePublisher            publisher  = null;
  private       Optional<DoubleSubscriber> subscriber = Optional.empty();


  public DoubleTelemetry(String keyString, double defaultVal)
  {
    key = keyString;
    defaultValue = defaultVal;
  }

  public void setupNetworkTables(NetworkTable dataTable, NetworkTable tuningTable)
  {
    var topic = dataTable.getDoubleTopic(key);
    publisher = topic.publish();
    publisher.setDefault(defaultValue);
    if (tuningTable != null)
    {
      subscriber = Optional.of(topic.subscribe(defaultValue));
    }
  }

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
        value = tuningValue;
        publisher.set(value);
        return false;
      }
    }
    publisher.accept(value);
    return true;
  }

  public double get()
  {
    if (subscriber.isPresent())
    {
      return subscriber.get().get(defaultValue);
    }
    throw new RuntimeException("Tuning table not configured for " + key + "!");
  }

  public boolean tunable()
  {
    return subscriber.isPresent();
  }

  public void enable()
  {
    enabled = true;
  }

  public void disable()
  {
    enabled = false;
  }

  public void display(boolean state)
  {
    enabled = state;
  }
}
