package yams.gearing;

import yams.exceptions.NoStagesGivenException;

/**
 * Sprocket class to handle calculating the conversion factor of a sprocket in your mechanism.
 */
public class Sprocket
{

  /**
   * Stages in the Sprocket chain.
   */
  private final double[] stages;
  /**
   * The input to output conversion factor.
   */
  private       double   conversionFactor;

  /**
   * Create the sprocket given the teeth of each sprocket in the chain.
   *
   * @param sprocketTeeth Sprocket teeth.
   */
  public Sprocket(double[] sprocketTeeth)
  {
    stages = sprocketTeeth;
    if (stages.length == 0)
    {
      throw new NoStagesGivenException();
    }
    if (stages.length % 2 != 0)
    {
      throw new RuntimeException("Not enough stages given, must be a multiple of 2!");
    }
    double sprocketRatio = stages[0] / stages[1]; // input / output
    for (int i = 2; i < stages.length; i += 2)
    {
      sprocketRatio *= stages[i] / stages[i + 1];
    }
    conversionFactor = sprocketRatio;
  }

  /**
   * Get the conversion factor to transform the sprocket input into the sprocket output rotations.
   *
   * @return OUT/IN or OUT:IN
   */
  public double getInputToOutputConversionFactor()
  {
    return conversionFactor;
  }

  /**
   * Get the conversion factor to transform the sprocket output value into the sprocket input value.
   *
   * @return IN:OUT or IN/OUT
   */
  public double getOutputToInputConversionFactor()
  {
    return 1 / conversionFactor;
  }


}
