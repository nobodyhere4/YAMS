package yams.telemetry;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

public class SmartMotorControllerTelemetryConfig {

    /**
     * Mechanism lower limit reached.
     */
    protected boolean mechanismLowerLimitEnabled = false;
    /**
     * Mechanism upper limit reached.
     */
    protected boolean mechanismUpperLimitEnabled = false;
    /**
     * Motor temperature cutoff reached.
     */
    protected boolean temperatureLimitEnabled = false;
    /**
     * Velocity PID controller used.
     */
    protected boolean velocityControlEnabled = false;
    /**
     * Elevator feedforward used.
     */
    protected boolean elevatorFeedforwardEnabled = false;
    /**
     * Arm feedforward used.
     */
    protected boolean armFeedforwardEnabled = false;
    /**
     * Simple feedforward used.
     */
    protected boolean simpleFeedforwardEnabled = false;
    /**
     * Motion profiling used.
     */
    protected boolean motionProfileEnabled = false;
    /**
     * Setpoint position given.
     */
    protected boolean setpointPositionEnabled = false;
    /**
     * Setpoint velocity given.
     */
    protected boolean setpointVelocityEnabled = false;
    /**
     * Feedforward voltage supplied to the {@link SmartMotorController}
     */
    protected boolean feedforwardVoltageEnabled = false;
    /**
     * PID Output voltage supplied to the {@link SmartMotorController}
     */
    protected boolean pidOutputVoltageEnabled = false;
    /**
     * Output voltage to the {@link SmartMotorController}
     */
    protected boolean outputVoltageEnabled = false;
    /**
     * Stator current (motor controller output current) to the Motor.
     */
    protected boolean statorCurrentEnabled = false;
    /**
     * Motor temperature.
     */
    protected boolean temperatureEnabled = false;
    /**
     * Mechanism distance.
     */
    protected boolean distanceEnabled = false;
    /**
     * Mechanism linear velocity.
     */
    protected boolean linearVelocityEnabled = false;
    /**
     * Mechanism position.
     */
    protected boolean mechanismPositionEnabled = false;
    /**
     * Mechanism velocity.
     */
    protected boolean mechanismVelocityEnabled = false;
    /**
     * Rotor position.
     */
    protected boolean rotorPositionEnabled = false;
    /**
     * Rotor velocity.
     */
    protected boolean rotorVelocityEnabled = false;

    /**
     * Setup with {@link TelemetryVerbosity}
     * @param verbosity {@link TelemetryVerbosity} to use.
     */
    public SmartMotorControllerTelemetryConfig withTelemetryVerbosity(TelemetryVerbosity verbosity)
    {
        switch (verbosity) {
            case HIGH:
                mechanismLowerLimitEnabled = true;
                mechanismUpperLimitEnabled = true;
                temperatureLimitEnabled = true;
                velocityControlEnabled = true;
                elevatorFeedforwardEnabled = true;
                armFeedforwardEnabled = true;
                simpleFeedforwardEnabled = true;
                motionProfileEnabled = true;
                setpointPositionEnabled = true;
                setpointVelocityEnabled = true;
                feedforwardVoltageEnabled = true;
                pidOutputVoltageEnabled = true;
                outputVoltageEnabled = true;
                statorCurrentEnabled = true;
                temperatureEnabled = true;
            case MID:
            case LOW:
                distanceEnabled = true;
                linearVelocityEnabled = true;
                mechanismPositionEnabled = true;
                mechanismVelocityEnabled = true;
                rotorPositionEnabled = true;
                rotorVelocityEnabled = true;
        }
        return this;
    }

    /**
     * Enables the mechanism lower limit logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismLowerLimit() {
        mechanismLowerLimitEnabled = true;
        return this;
    }

    /**
     * Enables the mechanism upper limit logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismUpperLimit() {
        mechanismUpperLimitEnabled = true;
        return this;
    }

    /**
     * Enables the temperature limit logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withTemperatureLimit() {
        temperatureLimitEnabled = true;
        return this;
    }

    /**
     * Enables the velocity control mode logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withVelocityControl() {
        velocityControlEnabled = true;
        return this;
    }

    /**
     * Enables the elevator feedforward logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withElevatorFeedforward() {
        elevatorFeedforwardEnabled = true;
        return this;
    }

    /**
     * Enables the arm feedforward logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withArmFeedforward() {
        armFeedforwardEnabled = true;
        return this;
    }

    /**
     * Enables the simple feedforward logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withSimpleFeedforward() {
        simpleFeedforwardEnabled = true;
        return this;
    }

    /**
     * Enables the motion profile logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMotionProfile() {
        motionProfileEnabled = true;
        return this;
    }

    /**
     * Enables the setpoint position logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withSetpointPosition() {
        setpointPositionEnabled = true;
        return this;
    }

    /**
     * Enables the setpoint velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withSetpointVelocity() {
        setpointVelocityEnabled = true;
        return this;
    }

    /**
     * Enables the feedforward voltage logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withFeedbackVoltage() {
        feedforwardVoltageEnabled = true;
        return this;
    }

    /**
     * Enables the pid output voltage logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withPidOutputVoltage() {
        pidOutputVoltageEnabled = true;
        return this;
    }

    /**
     * Enables the output voltage logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withOutputVoltage() {
        outputVoltageEnabled = true;
        return this;
    }

    /**
     * Enables the stator current logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withStatorCurrent() {
        statorCurrentEnabled = true;
        return this;
    }

    /**
     * Enables the temperature logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withTemperature() {
        temperatureEnabled = true;
        return this;
    }

    /**
     * Enables the distance logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withDistance() {
        distanceEnabled = true;
        return this;
    }

    /**
     * Enables the linear velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withLinearVelocity() {
        linearVelocityEnabled = true;
        return this;
    }

    /**
     * Enables the mechanism position logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismPosition() {
        mechanismPositionEnabled = true;
        return this;
    }

    /**
     * Enables the mechanism velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismVelocity() {
        mechanismVelocityEnabled = true;
        return this;
    }

    /**
     * Enables the rotor position logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withRotorPosition() {
        rotorPositionEnabled = true;
        return this;
    }

    /**
     * Enables the rotor velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withRotorVelocity() {
        rotorVelocityEnabled = true;
        return this;
    }
}
