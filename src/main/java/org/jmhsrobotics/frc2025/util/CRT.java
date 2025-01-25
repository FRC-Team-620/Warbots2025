package org.jmhsrobotics.frc2025.util;

public class CRT {
  // TODO Convert to double or more precise ints
  // TODO Scale to doubles
  private int leftGear;
  private int rightGear;
  private int encoderLeftValue;
  private int encoderRightValue;

  public CRT(int leftGear, int rightGear, int encoderLeftValue, int encoderRightValue) {
    this.leftGear = leftGear;
    this.rightGear = rightGear;
    this.encoderLeftValue = encoderLeftValue;
    this.encoderRightValue = encoderRightValue;
  }

  public double calcElevetorHeight() {
    int sum = 0;
    int temp;
    temp = GCDCalc(rightGear, leftGear);
    sum += encoderLeftValue * temp * rightGear;
    temp = GCDCalc(leftGear, rightGear);
    sum += encoderRightValue * temp * leftGear;

    return sum % (leftGear * rightGear);
  }

  // GCD of a and b is calculated using the equation GCD(a,b) = ax + by, were x and y are the bezout
  // coeeffiecients

  private int GCDCalc(int p, int q) {
    int s = 0; // quotient??
    int s_old = 1; // first Bezout Coefficient
    int t = 1; // quotient during algorithm
    int t_old = 0; // Bézout coefficient
    int r = q;
    int r_old = p; // greatest common divisor
    int quotient;
    int tmp;
    int x = 0;
    int y = 0;

    while (r != 0) { // do while r != 0
      quotient = r_old / (r);

      tmp = r; // temporarily store to update r, r_old simultaneously
      r = r_old - (quotient * r);
      r_old = tmp;

      tmp = s;
      s = s_old - (quotient * s);
      s_old = tmp;

      tmp = t;
      t = t_old - (quotient * t);
      t_old = tmp;
    }

    x = s_old; // x*p + y*q == gcd(p,q) ; this means x will be our b_i
    y = t_old;
    return x;
  }
}
