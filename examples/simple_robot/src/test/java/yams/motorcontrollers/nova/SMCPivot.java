package yams.motorcontrollers.nova;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SmartMotorControllerTestSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SchedulerPumpHelper;
import yams.helpers.TestWithScheduler;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;


public class SMCPivot
{

  private SmartMotorControllerTestSubsystem simpleSubsystem;
  private SmartMotorControllerConfig        config;
  private SmartMotorController              controller;
  private ThriftyNova                       motorController;
  private Pivot                             arm;

  @BeforeEach
  void startThread()
  {
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
    MockHardwareExtension.beforeAll();
    motorController = new ThriftyNova((int) (Math.random() * 50));
    simpleSubsystem = new SmartMotorControllerTestSubsystem();
    config = new SmartMotorControllerConfig(simpleSubsystem)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(0), Degrees.of(100))
        .withGearing(gearing(gearbox(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
    controller = new NovaWrapper(motorController, DCMotor.getNEO(1), config);
    PivotConfig m_config = new PivotConfig(controller)
        .withHardLimit(Degrees.of(-100), Degrees.of(200))
        .withStartingPosition(Degrees.of(0))
        .withMOI(0.001);
    arm = new Pivot(m_config);
    simpleSubsystem.smc = controller;
    simpleSubsystem.mechUpdateTelemetry = arm::updateTelemetry;
    simpleSubsystem.mechSimPeriodic = arm::simIterate;
    SimHooks.stepTiming(0.0); // Wait for Notifiers
    CommandScheduler.getInstance().enable();
    // teleopInit
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void stopThread() throws Exception
  {
    CommandScheduler.getInstance().unregisterSubsystem(simpleSubsystem);
    simpleSubsystem.close();
    motorController.close();
//        simPeriodicBeforeCallback.close();
//        simPeriodicAfterCallback.close();
    Preferences.removeAll();
    MockHardwareExtension.afterAll();
    TestWithScheduler.schedulerClear();

  }

  private void runScheduler(Time duration) throws InterruptedException
  {
    SchedulerPumpHelper.runForDuration(duration);
  }

  private void schedule(Command cmd)
  {
    CommandScheduler.getInstance().schedule(cmd);
  }

  @Test
  void testDutycycle() throws InterruptedException
  {
    Angle preDist = controller.getMechanismPosition();
    CommandScheduler.getInstance().schedule(simpleSubsystem.setDutyCycle(0.5));

    runScheduler(Seconds.of(3));

    System.out.println("PRE DIST: " + preDist);
    System.out.println("POST DIST: " + controller.getMechanismPosition());
    assertTrue(preDist.lt(controller.getMechanismPosition()));

    preDist = controller.getMechanismPosition();

    CommandScheduler.getInstance().schedule(simpleSubsystem.setDutyCycle(-1));
    runScheduler(Seconds.of(3));

    System.out.println("PRE DIST: " + preDist);
    System.out.println("POST DIST: " + controller.getMechanismPosition());
    assertTrue(preDist.gt(controller.getMechanismPosition()));
  }

  @Test
  void testPID() throws InterruptedException
  {
    Angle preDist = controller.getMechanismPosition();

    schedule(simpleSubsystem.setPositionSetpoint(Degrees.of(30)));
    runScheduler(Seconds.of(300));

    System.out.println("PRE DIST: " + preDist);
    System.out.println("POST DIST: " + controller.getMechanismPosition());
    assertTrue(preDist.lt(controller.getMechanismPosition()));

    preDist = controller.getMechanismPosition();

    schedule(simpleSubsystem.setPositionSetpoint(Degrees.of(0)));
    runScheduler(Seconds.of(300));

    System.out.println("PRE DIST: " + preDist);
    System.out.println("POST DIST: " + controller.getMechanismPosition());
    assertTrue(preDist.gt(controller.getMechanismPosition()));
  }

  @Test
  void testArmPID() throws InterruptedException
  {
    Angle preDist = controller.getMechanismPosition();

    schedule(arm.setAngle(Degrees.of(50)));
    runScheduler(Seconds.of(300));

    System.out.println("PRE DIST: " + preDist);
    System.out.println("POST DIST: " + controller.getMechanismPosition());
    assertTrue(preDist.lt(controller.getMechanismPosition()));

    preDist = controller.getMechanismPosition();

    schedule(arm.setAngle(Degrees.of(0)));
    runScheduler(Seconds.of(300));

    System.out.println("PRE DIST: " + preDist);
    System.out.println("POST DIST: " + controller.getMechanismPosition());
    assertTrue(preDist.gt(controller.getMechanismPosition()));
  }
}
