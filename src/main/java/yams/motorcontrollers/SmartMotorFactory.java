package yams.motorcontrollers;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.local.NovaWrapper;
import edu.wpi.first.math.system.plant.DCMotor;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkBase;
import com.thethriftybot.ThriftyNova;

public class SmartMotorFactory {
  @FunctionalInterface
  public static interface MotorControllerConstructor {
    public SmartMotorController create(Object... params);
  }

  public static final Map<String, MotorControllerConstructor> availableControllers = new HashMap<>();

  static {
    try {
      Class.forName("com.ctre.phoenix6.hardware.TalonFX");
      availableControllers.put("com.ctre.phoenix6.hardware.TalonFX", (Object... params) -> {
        TalonFX controller = (TalonFX) params[0];
        DCMotor motor = (DCMotor) params[1];
        SmartMotorControllerConfig smartConfig = (SmartMotorControllerConfig) params[2];
        return new TalonFXWrapper(controller, motor, smartConfig);
      });
    } catch (ClassNotFoundException ignored) {
    }

    try {
      Class.forName("com.ctre.phoenix6.hardware.TalonFXS");
      availableControllers.put("com.ctre.phoenix6.hardware.TalonFXS", (Object... params) -> {
        TalonFXS controller = (TalonFXS) params[0];
        DCMotor motor = (DCMotor) params[1];
        SmartMotorControllerConfig smartConfig = (SmartMotorControllerConfig) params[2];
        return new TalonFXSWrapper(controller, motor, smartConfig);
      });
    } catch (ClassNotFoundException ignored) {
    }

    try {
      Class.forName("com.revrobotics.spark.SparkBase");
      availableControllers.put("com.revrobotics.spark.SparkBase", (Object... params) -> {
        SparkBase controller = (SparkBase) params[0];
        DCMotor motor = (DCMotor) params[1];
        SmartMotorControllerConfig smartConfig = (SmartMotorControllerConfig) params[2];
        return new SparkWrapper(controller, motor, smartConfig);
      });
    } catch (ClassNotFoundException ignored) {
    }

    try {
      Class.forName("com.thethriftybot.ThriftyNova");
      availableControllers.put("com.thethriftybot.ThriftyNova", (Object... params) -> {
        ThriftyNova controller = (ThriftyNova) params[0];
        DCMotor motor = (DCMotor) params[1];
        SmartMotorControllerConfig smartConfig = (SmartMotorControllerConfig) params[2];
        return new NovaWrapper(controller, motor, smartConfig);
      });
    } catch (ClassNotFoundException ignored) {
    }
  }

  public static <T> Optional<SmartMotorController> create(Class<T> clazz, Object... params) {
    MotorControllerConstructor constructor = availableControllers.get(clazz.getName());
    return constructor != null ? Optional.of(constructor.create(params)) : Optional.empty();
  }
}
