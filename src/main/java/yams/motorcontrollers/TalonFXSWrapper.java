package yams.motorcontrollers;

import edu.wpi.first.math.system.plant.DCMotor;

public class TalonFXSWrapper implements SmartMotorController
{


  @Override
  public DCMotor getDCMotor()
  {
    return DCMotor.getNEO(1);
  }
}
