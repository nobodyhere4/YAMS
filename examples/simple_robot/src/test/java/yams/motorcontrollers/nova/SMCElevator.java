package yams.motorcontrollers.nova;

import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.NovaWrapper;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;


public class SMCElevator {
    private SmartMotorControllerTestSubsystem simpleSubsystem;
    private SmartMotorControllerConfig config;
    private SmartMotorController controller;
    private ThriftyNova motorController;
    private Elevator elevator;
    @BeforeEach
    void startThread() {
        TestWithScheduler.schedulerStart();
        TestWithScheduler.schedulerClear();
        MockHardwareExtension.beforeAll();
        motorController = new ThriftyNova((int)(Math.random()*50));
        simpleSubsystem = new SmartMotorControllerTestSubsystem();
        config = new SmartMotorControllerConfig(simpleSubsystem)
                .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
                .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
                .withSoftLimit(Meters.of(0), Meters.of(2))
                .withGearing(gearing(gearbox(3, 4)))
                .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(false)
                .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
                .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
        controller = new NovaWrapper(motorController, DCMotor.getNEO(1), config);
        ElevatorConfig m_config      = new ElevatorConfig(controller)
                .withStartingHeight(Meters.of(0.5))
                .withHardLimits(Meters.of(0), Meters.of(3))
                .withMass(Pounds.of(16));
        elevator = new Elevator(m_config);
        simpleSubsystem.smc = controller;
        simpleSubsystem.mechUpdateTelemetry = elevator::updateTelemetry;
        simpleSubsystem.mechSimPeriodic = elevator::simIterate;
        SimHooks.stepTiming(0.0); // Wait for Notifiers
        CommandScheduler.getInstance().enable();
        // teleopInit
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void stopThread() throws Exception {
        CommandScheduler.getInstance().unregisterSubsystem(simpleSubsystem);
        simpleSubsystem.close();
        motorController.close();
//        simPeriodicBeforeCallback.close();
//        simPeriodicAfterCallback.close();
        Preferences.removeAll();
        MockHardwareExtension.afterAll();
        TestWithScheduler.schedulerClear();

    }

    private void runScheduler(Time duration) throws InterruptedException {
        SchedulerPumpHelper.runForDuration(duration);
    }

    private void schedule(Command cmd)
    {
        CommandScheduler.getInstance().schedule(cmd);
    }

    @Test
    void testDutycycle() throws InterruptedException{
        Distance preDist = controller.getMeasurementPosition();
        CommandScheduler.getInstance().schedule(simpleSubsystem.setDutyCycle(0.5));

        runScheduler(Seconds.of(3));

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.lt(controller.getMeasurementPosition()));

        preDist = controller.getMeasurementPosition();

        CommandScheduler.getInstance().schedule(simpleSubsystem.setDutyCycle(-1));
        runScheduler(Seconds.of(3));

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.gt(controller.getMeasurementPosition()));
    }

    @Test
    void testPID() throws InterruptedException {
        Distance preDist = controller.getMeasurementPosition();

        schedule(simpleSubsystem.setPositionSetpoint(Meters.of(3)));
        runScheduler(Seconds.of(3));

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.lt(controller.getMeasurementPosition()));

        preDist = controller.getMeasurementPosition();

        schedule(simpleSubsystem.setPositionSetpoint(Meters.of(0)));
        runScheduler(Seconds.of(10));

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.gt(controller.getMeasurementPosition()));
    }

    @Test
    void testElevatorPID() throws InterruptedException {
        Distance preDist = controller.getMeasurementPosition();

        schedule(elevator.setHeight(Meters.of(3)));
        runScheduler(Seconds.of(3));

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.lt(controller.getMeasurementPosition()));

        preDist = controller.getMeasurementPosition();

        schedule(elevator.setHeight(Meters.of(0)));
        runScheduler(Seconds.of(10));

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.gt(controller.getMeasurementPosition()));
    }
}
