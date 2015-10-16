/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.math.BigInteger;

/**
 * WARNING: DO NOT USE: IT IS NOT FINISHED AND WILL CRASH
 * 
 * A class to calculate PI. It was going to save the state of the calculation
 * between runs and resume where it left off, but then I realized I was wasting
 * my time and I had more important things to do.
 * 
 * @author Ben Wolsieffer
 */
public class PiCalculator {

    private final BufferedWriter outputFileWriter;
    private State state = new State();

    private final BigInteger TWO = BigInteger.valueOf(2);
    private final BigInteger THREE = BigInteger.valueOf(3);
    private final BigInteger FOUR = BigInteger.valueOf(4);
    private final BigInteger SEVEN = BigInteger.valueOf(7);

    private final class State {

        public BigInteger q = BigInteger.ONE;
        public BigInteger r = BigInteger.ZERO;
        public BigInteger t = BigInteger.ONE;
        public BigInteger k = BigInteger.ONE;
        public BigInteger n = THREE;
        public BigInteger l = THREE;

        public BigInteger nn, nr;
        public boolean first = true;
    }

    public PiCalculator(File outputFile) throws IOException {
        outputFile.createNewFile();
        outputFileWriter = new BufferedWriter(new FileWriter(outputFile, true));
    }

    public PiCalculator(File stateFile, File outputFile) throws IOException {
        this(outputFile);
        stateFile.createNewFile();
    }

    public void calcPiDigit() throws IOException {
        if (FOUR.multiply(state.q).add(state.r).subtract(state.t).compareTo(
                state.n.multiply(state.t)) == -1) {
            outputFileWriter.write(state.n.toString());
            if (state.first) {
                outputFileWriter.write('.');
                state.first = false;
            }
            state.nr = BigInteger.TEN.multiply(state.r.subtract(state.n.multiply(state.t)));
            state.n = BigInteger.TEN.multiply(THREE.multiply(state.q).add(state.r)).divide(
                    state.t).subtract(BigInteger.TEN.multiply(state.n));
            state.q = state.q.multiply(BigInteger.TEN);
            state.r = state.nr;
        } else {
            state.nr = TWO.multiply(state.q).add(state.r).multiply(state.l);
            state.nn = state.q.multiply((SEVEN.multiply(state.k))).add(TWO).add(
                    state.r.multiply(state.l)).divide(state.t.multiply(state.l));
            state.q = state.q.multiply(state.k);
            state.t = state.t.multiply(state.l);
            state.l = state.l.add(TWO);
            state.k = state.k.add(BigInteger.ONE);
            state.n = state.nn;
            state.r = state.nr;
        }
    }
}
