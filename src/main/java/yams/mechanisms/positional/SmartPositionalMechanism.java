package yams.mechanisms.positional;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import yams.mechanisms.SmartMechanism;

/**
 * Generic for positional mechanisms.
 */
public abstract class SmartPositionalMechanism extends SmartMechanism
{

  /**
   * The root point of the Mechanism.
   */
  protected MechanismRoot2d     mechanismRoot;
  /**
   * The ligament that is being moved.
   */
  protected MechanismLigament2d mechanismLigament;
}
