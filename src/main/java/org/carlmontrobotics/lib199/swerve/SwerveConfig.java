package org.carlmontrobotics.lib199.swerve;

public final class SwerveConfig {

    public double wheelDiameterMeters, driveGearing, mu, autoCentripetalAccel, driveModifier;
    public double[] kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels, kBackwardAccels,
            drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZero;
    public boolean[] driveInversion, turnInversion, reversed;

    public SwerveConfig(double wheelDiameterMeters, double driveGearing, double mu,
            double autoCentripetalAccel, double[] kForwardVolts, double[] kForwardVels, double[] kForwardAccels,
            double[] kBackwardVolts, double[] kBackwardVels, double[] kBackwardAccels, double[] drivekP,
            double[] drivekI, double[] drivekD, double[] turnkP, double[] turnkI, double[] turnkD, double[] turnkS,
            double[] turnkV, double[] turnkA, double[] turnZero, boolean[] driveInversion, boolean[] reversed, double driveModifier, boolean[] turnInversion) {
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.driveGearing = driveGearing;
        this.mu = mu;//coefficient of friction between the wheel and the surface
        this.autoCentripetalAccel = autoCentripetalAccel;
        this.kForwardVolts = kForwardVolts;
        this.kForwardVels = kForwardVels;
        this.kForwardAccels = kForwardAccels;
        this.kBackwardVolts = kBackwardVolts;
        this.kBackwardVels = kBackwardVels;
        this.kBackwardAccels = kBackwardAccels;
        this.drivekP = drivekP;
        this.drivekI = drivekI;
        this.drivekD = drivekD;
        this.turnkP = turnkP;
        this.turnkI = turnkI;
        this.turnkD = turnkD;
        this.turnkS = turnkS;//for overcoming static friction
        this.turnkV = turnkV;
        this.turnkA = turnkA;
        this.turnZero = turnZero;
        this.driveInversion = driveInversion;
        this.reversed = reversed;
        this.driveModifier = driveModifier;
        this.turnInversion = turnInversion;
    }

}
