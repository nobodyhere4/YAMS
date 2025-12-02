package frc.robot.subsystems.doubleflywheel;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class LowerFlyWheelSubsystem extends SubsystemBase
{

  private       SparkMax                   smax                       = new SparkMax(1, MotorType.kBrushless);
  private final SmartMotorControllerConfig smartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withTelemetry("SmatMotorController Name Goes HERE", TelemetryVerbosity.HIGH)
      .withTelemetry("NAME ME", new SmartMotorControllerTelemetryConfig()
          .withTelemetryVerbosity(TelemetryVerbosity.LOW)
          .withArmFeedforward()
          .withSimpleFeedforward()
          .withElevatorFeedforward()
          .withMotionProfile()
          .withSetpointPosition()
          .withMechanismPosition())

      .withExternalEncoder(smax.getAbsoluteEncoder())
      .withExternalEncoderInverted(false)
      .withExternalEncoderGearing(new MechanismGearing(0.5)) // 1:2 -> input:output
      .withExternalEncoderZeroOffset(Degrees.of(0))
      .withUseExternalFeedbackEncoder(true)

      .withWheelRadius(Inches.of(2))
      .withClosedLoopController(0, 0, 0, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(1))

      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withMechanismCircumference(Inches.of(4).times(Math.PI))
      .withWheelDiameter(Inches.of(4))
      .withWheelRadius(Inches.of(2))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(1, 0, 0, RPM.of(10000), RPM.per(Second).of(60))
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withMotorInverted(false)
      .withGearing(21)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withTelemetry("LowerFlywheel", TelemetryVerbosity.HIGH);
  private       SmartMotorController       motor;
  private CANcoder cancoder = new CANcoder(1);

  public LowerFlyWheelSubsystem() throws InterruptedException
  {

    motor = new SparkWrapper(new SparkMax(1, MotorType.kBrushless),
                             DCMotor.getNEO(1),
                             smartMotorControllerConfig);
    motor.getMeasurementPosition().in(Meters);
    motor.getMeasurementVelocity().in(MetersPerSecond);

    DCMotor minion = new DCMotor(12,
                                 3.1,
                                 200.46,
                                 1.43,
                                 RPM.of(7200).in(RadiansPerSecond), // freeSpeedRadPerSec
                                 1); // numMotors
    motor = new TalonFXSWrapper(new TalonFXS(1),
                                minion,
                                smartMotorControllerConfig);

    motor = new NovaWrapper(new ThriftyNova(1),
                            DCMotor.getNEO(1),
                            smartMotorControllerConfig);

    motor = new TalonFXWrapper(new TalonFX(1),
                               DCMotor.getKrakenX60(1),
                               smartMotorControllerConfig);

    motor.setPosition(Degrees.of(0));
    motor.stopClosedLoopController();
    motor.setVoltage(Volts.of(4));
    Thread.sleep(1000);
    motor.startClosedLoopController();

    // Set the motor position to 0deg
    run(() -> motor.setPosition(Degrees.zero()));

    // Move the motor with 4 volts for 1 second
    startRun(() -> {motor.stopClosedLoopController();},
             () -> {motor.setVoltage(Volts.of(4));})
        .withTimeout(Seconds.of(1))
        .finallyDo(() -> {motor.startClosedLoopController();});
    // Stop the closed loop controller at the start, then start it at the end
    // Motor will go back to previous setpoint (0deg) after command ends.

    motor.setPosition(Degrees.of(30));
    motor.setPosition(Meters.of(3));

    motor.setVelocity(RPM.of(10000));
    motor.setVelocity(MetersPerSecond.of(1));

    motor.setDutyCycle(0.5);

    motor.setVoltage(Volts.of(7));

    Angle absPosition = cancoder.getAbsolutePosition().getValue();
    MechanismGearing gearing = new MechanismGearing(0.5);
    motor.setEncoderPosition(absPosition.times(gearing.getRotorToMechanismRatio()));
    // If no gearing is required
    motor.setEncoderPosition(absPosition);

  }

  public void setPosition(Angle position)
  {
  }

  @Override
  public void periodic()
  {
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    motor.simIterate();
  }
}

