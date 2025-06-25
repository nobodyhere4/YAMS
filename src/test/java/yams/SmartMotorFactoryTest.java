package yams;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkBase;
import com.thethriftybot.ThriftyNova;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorFactory;
import yams.motorcontrollers.SmartMotorControllerConfig;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;

import edu.wpi.first.math.system.plant.DCMotor;

public class SmartMotorFactoryTest {

  @BeforeAll
  static void setupMockFactories() {
    SmartMotorFactory.availableControllers.put("com.ctre.phoenix6.hardware.TalonFX",
        (params) -> mock(SmartMotorController.class));
    SmartMotorFactory.availableControllers.put("com.ctre.phoenix6.hardware.TalonFXS",
        (params) -> mock(SmartMotorController.class));
    SmartMotorFactory.availableControllers.put("com.revrobotics.spark.SparkBase",
        (params) -> mock(SmartMotorController.class));
    SmartMotorFactory.availableControllers.put("com.thriftybots.nova.ThriftyNova",
        (params) -> mock(SmartMotorController.class));
  }

  @Test
  void testFactoryRegistration() {
    assertFalse(SmartMotorFactory.availableControllers.isEmpty(), "Factory should have registered controllers");
  }

  @Test
  void testCreateTalonFXWrapper() {
    TalonFX mockController = mock(TalonFX.class);
    DCMotor mockMotor = createMockDCMotor();
    SmartMotorControllerConfig mockConfig = createMockSmartConfig();

    Optional<SmartMotorController> result = SmartMotorFactory.create(TalonFX.class, mockController, mockMotor,
        mockConfig);

    assertTrue(result.isPresent(), "Factory should create a TalonFX SmartMotorController");
  }

  @Test
  void testCreateTalonFXSWrapper() {
    TalonFXS mockController = mock(TalonFXS.class);
    DCMotor mockMotor = createMockDCMotor();
    SmartMotorControllerConfig mockConfig = createMockSmartConfig();

    Optional<SmartMotorController> result = SmartMotorFactory.create(TalonFXS.class, mockController, mockMotor,
        mockConfig);

    assertTrue(result.isPresent(), "Factory should create a TalonFXS SmartMotorController");
  }

  @Test
  void testCreateSparkWrapper() {
    SparkBase mockController = mock(SparkBase.class);
    DCMotor mockMotor = createMockDCMotor();
    SmartMotorControllerConfig mockConfig = createMockSmartConfig();

    Optional<SmartMotorController> result = SmartMotorFactory.create(SparkBase.class, mockController, mockMotor,
        mockConfig);

    assertTrue(result.isPresent(), "Factory should create a Spark SmartMotorController");
  }

  @Test
  void testCreateNovaWrapper() {
    ThriftyNova mockController = mock(ThriftyNova.class);
    DCMotor mockMotor = createMockDCMotor();
    SmartMotorControllerConfig mockConfig = createMockSmartConfig();

    Optional<SmartMotorController> result = SmartMotorFactory.create(ThriftyNova.class, mockController, mockMotor,
        mockConfig);

    assertTrue(result.isPresent(), "Factory should create a Nova SmartMotorController");
  }

  @Test
  void testCreateUnsupportedClassReturnsEmpty() {
    class DummyController {
    }

    Optional<SmartMotorController> result = SmartMotorFactory.create(DummyController.class, new DummyController(), null,
        null);
    assertFalse(result.isPresent(), "Factory should return empty for unsupported classes");
  }

  private static DCMotor createMockDCMotor() {
    DCMotor motor = mock(DCMotor.class);
    return motor;
  }

  private static SmartMotorControllerConfig createMockSmartConfig() {
    SmartMotorControllerConfig config = mock(SmartMotorControllerConfig.class);
    return config;
  }
}
