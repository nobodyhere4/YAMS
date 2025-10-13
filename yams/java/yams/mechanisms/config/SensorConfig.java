package yams.mechanisms.config;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import yams.motorcontrollers.simulation.SensorData;
import yams.motorcontrollers.simulation.SensorSim;

/**
 * Sensor configuration for simulated and real sensors.
 */
public class SensorConfig
{

  /**
   * Sensor name to display in the simulation window.
   */
  private final String              name;
  /**
   * List of {@link SensorData} to display in the simulation window.
   */
  private       List<SensorData>    data   = new ArrayList<>();
  /**
   * Sensor
   */
  private       Optional<SensorSim> sensor = Optional.empty();

  /**
   * Sensor configuration.
   *
   * @param name Name of sensor to display in the simulation window.
   */
  public SensorConfig(String name)
  {
    this.name = name;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, DoubleSupplier supplier, double defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, IntSupplier supplier, int defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, BooleanSupplier supplier, boolean defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, LongSupplier supplier, long defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Get the {@link SensorSim} for this sensor.
   *
   * @return {@link SensorSim} for fetching real and simulated values.
   */
  public SensorSim getSensor()
  {
    if (sensor.isEmpty())
    {
      sensor = Optional.of(new SensorSim(name, data));
    }
    return sensor.get();
  }

}
