package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {

  // ========Driver Controls========
  public double rotation();

  public double translationX();

  public double translationY();

  public Trigger resetForward();

  public Trigger upExample();

  public Trigger downExample();

  // =======Operator Controls=======

}
