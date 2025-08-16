package yams.mechs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SmartMotorControllerTestSubsystem;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import yams.helpers.MockHardwareExtension;
import yams.helpers.TestWithScheduler;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class PivotTest
{

  private static SmartMotorControllerConfig createSMCConfig()
  {
    SmartMotorControllerTestSubsystem subsystem = new SmartMotorControllerTestSubsystem();

    return new SmartMotorControllerConfig(subsystem)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(-100), Degrees.of(100))
        .withGearing(gearing(gearbox(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0, 0.02))
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private static Pivot createPivot(SmartMotorController smc)
  {
    PivotConfig config = new PivotConfig(smc)
        .withHardLimit(Degrees.of(-100), Degrees.of(200))
        .withStartingPosition(Degrees.of(0))
        .withMOI(0.001);
    Pivot                             pivot  = new Pivot(config);
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.smc = smc;
    subsys.mechSimPeriodic = pivot::simIterate;
    subsys.mechUpdateTelemetry = pivot::updateTelemetry;
    return pivot;
  }

  private static Stream<Arguments> createConfigs()
  {
    SparkMax    smax  = new SparkMax((int) (Math.random() * 50), MotorType.kBrushless);
    SparkFlex   sflex = new SparkFlex((int) (Math.random() * 50), MotorType.kBrushless);
    ThriftyNova tnova = new ThriftyNova((int) (Math.random() * 50));
    TalonFXS    tfxs  = new TalonFXS((int) (Math.random() * 50));
    TalonFX     tfx   = new TalonFX((int) (Math.random() * 50));

    return Stream.of(
        Arguments.of(setupTestSubsystem(new SparkWrapper(smax, DCMotor.getNEO(1),
                                                         createSMCConfig()
                                                             .withClosedLoopRampRate(Seconds.of(0.25))
                                                             .withOpenLoopRampRate(Seconds.of(0.25))))),
        Arguments.of(setupTestSubsystem(new SparkWrapper(sflex, DCMotor.getNeoVortex(1),
                                                         createSMCConfig()
                                                             .withClosedLoopRampRate(Seconds.of(0.25))
                                                             .withOpenLoopRampRate(Seconds.of(0.25))))),
        Arguments.of(setupTestSubsystem(new NovaWrapper(tnova, DCMotor.getNEO(1), createSMCConfig()))),
        Arguments.of(setupTestSubsystem(new TalonFXSWrapper(tfxs, DCMotor.getNEO(1),
                                                            createSMCConfig()
                                                                .withClosedLoopRampRate(Seconds.of(0.25))
                                                                .withOpenLoopRampRate(Seconds.of(0.25))))),
        Arguments.of(setupTestSubsystem(new TalonFXWrapper(tfx, DCMotor.getNEO(1),
                                                           createSMCConfig()
                                                               .withClosedLoopRampRate(Seconds.of(0.25))
                                                               .withOpenLoopRampRate(Seconds.of(0.25)))))
                    );
  }


  private static void closeSMC(SmartMotorController smc)
  {
    CommandScheduler.getInstance().unregisterSubsystem((SmartMotorControllerTestSubsystem) smc.getConfig()
                                                                                              .getSubsystem());
    ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).close();

//    switch (smc.getMotorController())
//    {
//
//    }
    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax)
    {
      ((SparkMax) motorController).close();
    } else if (motorController instanceof SparkFlex)
    {
      ((SparkFlex) motorController).close();
    } else if (motorController instanceof ThriftyNova)
    {
//      ((ThriftyNova)motorController).close();
    } else if (motorController instanceof TalonFXS)
    {
      ((TalonFXS) motorController).close();
    } else if (motorController instanceof TalonFX)
    {
      ((TalonFX) motorController).close();
    }
  }

  private static void positionPidTest(SmartMotorController smc, Command highPIDSetCommand, Command lowPIDSetCommand)
  throws InterruptedException
  {
    Angle pre = smc.getMechanismPosition();
    Angle post;

    TestWithScheduler.schedule(highPIDSetCommand);
    TestWithScheduler.cycle(Seconds.of(300));

    post = smc.getMechanismPosition();
    System.out.println("PID High PreTest Angle: " + pre);
    System.out.println("PID High PostTest Angle: " + post);
    assertTrue(pre.lt(post));

    pre = smc.getMechanismPosition();
    TestWithScheduler.schedule(lowPIDSetCommand);
    TestWithScheduler.cycle(Seconds.of(300));

    post = smc.getMechanismPosition();
    System.out.println("PID Low PreTest Angle: " + pre);
    System.out.println("PID Low PostTest Angle: " + post);
    assertTrue(pre.gt(post));
  }

  private static void dutyCycleTest(SmartMotorController smc, Command dutycycleUp, Command dutyCycleDown)
  throws InterruptedException
  {
    Angle pre = smc.getMechanismPosition();
    Angle post;

    TestWithScheduler.schedule(dutycycleUp);
    TestWithScheduler.cycle(Seconds.of(30));

    post = smc.getMechanismPosition();
    System.out.println("DutyCycleUp PreTest Angle: " + pre);
    System.out.println("DutyCycleUp PostTest Angle: " + post);
    assertTrue(pre.lt(post));

    pre = smc.getMechanismPosition();
    TestWithScheduler.schedule(dutyCycleDown);
    TestWithScheduler.cycle(Seconds.of(30));

    post = smc.getMechanismPosition();
    System.out.println("DutyCycleDown PreTest Angle: " + pre);
    System.out.println("DutyCycleDown PostTest Angle: " + post);
    assertTrue(pre.gt(post));
  }

  private static SmartMotorController setupTestSubsystem(SmartMotorController smc)
  {
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.setSMC(smc);
    return smc;
  }

  private static void startTest(SmartMotorController smc)
  {
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.testRunning = true;
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testSMCDutyCycle(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();

    Command dutyCycleUp   = subsys.setDutyCycle(0.5);
    Command dutyCycleDown = subsys.setDutyCycle(-0.5);

    dutyCycleTest(smc, dutyCycleUp, dutyCycleDown);

    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testPivotDutyCycle(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Pivot   pivot         = createPivot(smc);
    Command dutyCycleUp   = pivot.set(0.5);
    Command dutyCycleDown = pivot.set(-0.5);

    dutyCycleTest(smc, dutyCycleUp, dutyCycleDown);

    closeSMC(smc);
  }


  @ParameterizedTest
  @MethodSource("createConfigs")
  void testSMCPositionPID(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Command highPid = Commands.run(() -> smc.setPosition(Degrees.of(80)));
    Command lowPid  = Commands.run(() -> smc.setPosition(Degrees.of(-80)));

    positionPidTest(smc, highPid, lowPid);

    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testPivotPositionPID(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Pivot   pivot   = createPivot(smc);
    Command highPid = pivot.setAngle(Degrees.of(80));
    Command lowPid  = pivot.setAngle(Degrees.of(-80));

    positionPidTest(smc, highPid, lowPid);

    closeSMC(smc);
  }

  @BeforeEach
  void startTest()
  {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void endTest()
  {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
