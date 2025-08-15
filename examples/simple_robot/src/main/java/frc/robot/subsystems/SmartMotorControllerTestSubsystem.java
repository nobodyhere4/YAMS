package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;

public class SmartMotorControllerTestSubsystem extends SubsystemBase {
    public SmartMotorController smc;

    public SmartMotorControllerTestSubsystem() {
    }

    public Command setDutyCycle(double dutyCycle)
    {
        return runOnce(smc::stopClosedLoopController)
                .andThen(run(()->{smc.setDutyCycle(dutyCycle);System.out.println("RUNNING!");}))
                .finallyDo(smc::startClosedLoopController);
    }

    public Command setPositionSetpoint(Angle position)
    {
        return run(()->smc.setPosition(position));
    }

    public Command setPositionSetpoint(Distance position)
    {
        return run(()->smc.setPosition(position));
    }

    public void close()
    {
        smc.close();
    }

    @Override
    public void periodic() {
        smc.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        smc.simIterate();
    }
}
