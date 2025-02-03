package org.jmhsrobotics.frc2025.util;

import java.math.BigInteger;

public class oldcrt {
  // TODO Convert to double or more precise ints
  // TODO Scale to doubles
  private int leftGear;
  private int rightGear;
  private int encoderLeftValue;
  private int encoderRightValue;

  public oldcrt(int leftGear, int rightGear, int encoderLeftValue, int encoderRightValue) {
    this.leftGear = leftGear;
    this.rightGear = rightGear;
    this.encoderLeftValue = encoderLeftValue;
    this.encoderRightValue = encoderRightValue;
  }

  public double calcElevetorHeight() {
    int sum = 0;
    int temp;
    temp =
        extended_euclidean_algortihm(new BigInteger(rightGear + ""), new BigInteger(leftGear + ""))
            .intValue();
    // GCDCalc(rightGear, leftGear);
    sum += encoderLeftValue * temp * rightGear;
    temp =
        extended_euclidean_algortihm(new BigInteger(leftGear + ""), new BigInteger(rightGear + ""))
            .intValue();
    ;
    // GCDCalc(leftGear, rightGear);
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

  public static BigInteger extended_euclidean_algortihm(BigInteger p, BigInteger q) {
    BigInteger s = new BigInteger("0"); // quotient during algorithm
    BigInteger s_old = new BigInteger("1"); // Bézout coefficient
    BigInteger t = new BigInteger("1"); // quotient during algorithm
    BigInteger t_old = new BigInteger("0"); // Bézout coefficient
    BigInteger r = q;
    BigInteger r_old = p; // greatest common divisor
    BigInteger quotient;
    BigInteger tmp;

    while (r.compareTo(BigInteger.valueOf(0)) != 0) { // do while r != 0
      quotient = r_old.divide(r);

      tmp = r; // temporarily store to update r, r_old simultaneously
      r = r_old.subtract(quotient.multiply(r));
      r_old = tmp;

      tmp = s;
      s = s_old.subtract(quotient.multiply(s));
      s_old = tmp;

      tmp = t;
      t = t_old.subtract(quotient.multiply(t));
      t_old = tmp;
    }

    // x = ; // x*p + y*q == gcd(p,q) ; this means x will be our b_i
    // y = t_old;
    return s_old;
  }
}
