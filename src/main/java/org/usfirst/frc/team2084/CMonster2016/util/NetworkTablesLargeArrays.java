/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.util;

import java.nio.ByteBuffer;
import java.util.Base64;

import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Ben Wolsieffer
 */
public class NetworkTablesLargeArrays {

    @SuppressWarnings("serial")
    public static class EncodedLengthException extends RuntimeException {

        public EncodedLengthException(String key) {
            super("Encoded array \"" + key + "\" exceeded the maximum length of " + MAX_ENCODED_LENGTH);
        }
    }

    private static final Base64.Encoder encoder = Base64.getEncoder();

    private static final int MAX_ENCODED_LENGTH = (int) Math.pow(2, Short.SIZE);

    public static void putRaw(ITable table, String key, byte[] value) {
        String b64 = encoder.encodeToString(value);
        if (b64.length() <= MAX_ENCODED_LENGTH) {
            table.putString(key, b64);
        } else {
            throw new EncodedLengthException(key);
        }
    }

    public static void putNumberArray(ITable table, String key, double[] value) {
        ByteBuffer valueBuffer = ByteBuffer.allocate(value.length * Double.BYTES);
        for (int i = 0; i < value.length; i++) {
            valueBuffer.putDouble(value[i]);
        }
        putRaw(table, key, valueBuffer.array());
    }
}
