package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {

  // ========Driver Controls========
  public double rotation();

  public double translationX();

  public double translationY();

  public Trigger resetForward();

  public Trigger turboMode();

  public Trigger upExample();

  public Trigger downExample();

  // =======Operator Controls=======

  public Trigger intakeCoral();

  public Trigger extakeCoral();

  public Trigger placeCoralL1();

  public Trigger placeCoralL2();

  public Trigger placeCoralL3();

  public Trigger placeCoralL4();

  public Trigger removeAlgaeL23();

  public Trigger removeAlgaeL34();

  public Trigger scoreProcessor();

  public Trigger scoreBarge();

  public Trigger climbUp();

  public Trigger climbDown();

  public Trigger indexerUp();

  public Trigger indexerDown();
}
