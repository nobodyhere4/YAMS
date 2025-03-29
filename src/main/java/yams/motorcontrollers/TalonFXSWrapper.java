package yams.motorcontrollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public class TalonFXSWrapper implements SmartMotorController
{


  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {

  }

  @Override
  public void setVelocity(AngularVelocity velocity)
  {

  }

  @Override
  public void setVelocity(LinearVelocity velocity)
  {

  }

  @Override
  public void setPosition(Angle angle)
  {

  }

  @Override
  public void setPosition(Distance distance)
  {

  }

  @Override
  public Command sysId(VoltageUnit maxVoltage, VelocityUnit<VoltageUnit> stepVoltage, TimeUnit testDuration)
  {
    return null;
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    return false;
  }

  @Override
  public double getDutyCycle()
  {
    return 0;
  }

  @Override
  public Current getSupplyCurrent()
  {
    return null;
  }

  @Override
  public Current getStatorCurrent()
  {
    return null;
  }

  @Override
  public Voltage getVoltage()
  {
    return null;
  }

  @Override
  public DCMotor getDCMotor()
  {
    return DCMotor.getNEO(1);
  }

  @Override
  public LinearVelocity getMeasurementVelocity()
  {
    return null;
  }

  @Override
  public Distance getMeasurementPosition()
  {
    return null;
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    return null;
  }

  @Override
  public Angle getMechanismPosition()
  {
    return null;
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return null;
  }

  @Override
  public Angle getRotorPosition()
  {
    return null;
  }
}
