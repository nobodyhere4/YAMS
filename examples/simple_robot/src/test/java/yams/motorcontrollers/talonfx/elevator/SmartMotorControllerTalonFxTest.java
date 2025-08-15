package yams.motorcontrollers.talonfx.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.SmartMotorControllerTestSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;


public class SmartMotorControllerTalonFxTest {
    private SmartMotorControllerTestSubsystem simpleSubsystem;
    private SmartMotorControllerConfig config;
    private SmartMotorController controller;
    private TalonFX talonFX;
    private HAL.SimPeriodicAfterCallback simPeriodicAfterCallback;
    private HAL.SimPeriodicBeforeCallback simPeriodicBeforeCallback;
    @BeforeEach
    void startThread() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        talonFX = new TalonFX(54);
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
        controller = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), config);
//        simPeriodicAfterCallback = HAL.registerSimPeriodicAfterCallback(simpleSubsystem::periodic);
//        simPeriodicBeforeCallback = HAL.registerSimPeriodicBeforeCallback(simpleSubsystem::simulationPeriodic);
        simpleSubsystem.smc = controller;
        SimHooks.stepTiming(0.0); // Wait for Notifiers

        // teleopInit
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void stopThread() {
        simpleSubsystem.close();
        talonFX.close();
//        simPeriodicBeforeCallback.close();
//        simPeriodicAfterCallback.close();
        Preferences.removeAll();
        RoboRioSim.resetData();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    private void runCommand(double seconds, Runnable cmd)
    {
        double timer = System.currentTimeMillis();

        while ((System.currentTimeMillis()-timer) < seconds*1000) {
            Timer.delay(0.02);
            cmd.run();
            simpleSubsystem.simulationPeriodic();
            simpleSubsystem.periodic();
        }
    }

    @Test
    void testDutycycle() {
        // advance 150 timesteps
        SimHooks.stepTiming(3);

        Distance preDist = controller.getMeasurementPosition();
        runCommand(2,()->controller.setDutyCycle(0.5));
        SimHooks.stepTiming(2);

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.lt(controller.getMeasurementPosition()));

        preDist = controller.getMeasurementPosition();

        runCommand(3,()->controller.setDutyCycle(-1));
        SimHooks.stepTiming(3);

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.gt(controller.getMeasurementPosition()));
    }

    @Test
    void testPID()
    {
        // advance 150 timesteps
        SimHooks.stepTiming(3);

        Distance preDist = controller.getMeasurementPosition();
        runCommand(2,()->controller.setPosition(Meters.of(1)));
        SimHooks.stepTiming(2);

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.lt(controller.getMeasurementPosition()));

        preDist = controller.getMeasurementPosition();

        runCommand(3,()->controller.setPosition(Meters.of(0)));
        SimHooks.stepTiming(3);

        System.out.println("PRE DIST: " + preDist);
        System.out.println("POST DIST: " + controller.getMeasurementPosition());
        assertTrue(preDist.gt(controller.getMeasurementPosition()));
    }
}
