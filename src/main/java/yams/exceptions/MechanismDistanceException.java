package yams.exceptions;

public class MechanismDistanceException extends IllegalArgumentException
{

  public MechanismDistanceException()
  {
    super(
        "Mechanism to distance ratio is not defined! Please configure it with 'SmartMotionControllerConfig.withMechanismCircumference(Inches.of(3*Math.Pi))'");
  }
}
