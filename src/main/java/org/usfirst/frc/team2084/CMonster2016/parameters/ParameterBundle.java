/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.parameters;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Properties;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Class for interacting with parameters on the web interface. These objects are
 * often created on a per class basis, but may be shared between multiple
 * classes that need to share the same parameter values.
 * 
 * @author Ben Wolsieffer
 */
public class ParameterBundle<T> {

    /**
     * Exception thrown when a parameter is accessed that does not exist.
     */
    @SuppressWarnings("serial")
    public static class ParameterNotFoundException extends RuntimeException {

        public ParameterNotFoundException(String key) {
            super("Could not find parameter with key: " + key);
        }
    }

    /**
     * Interface for subscribing to parameter changes.
     */
    public static interface Listener {

        void parameterChanged(String p, Type type, Object value);
    }

    /**
     * Cache of parameter list for loaded classes. This prevents having to call
     * reflection more than necessary, but I don't know if it is actually
     * faster.
     */
    private static final HashMap<Class<?>, Parameter[]> parameterCache = new HashMap<>();

    private static final ITable PARAMETER_TABLE = NetworkTable.getTable("Parameters");

    private final ITable table;
    private final HashMap<String, Parameter> parameters;

    /**
     * Creates a parameter bundle for the specified class with the specified
     * name. This is the most commonly used constructor. This looks for defaults
     * in the "parameters" folder of the JAR.
     * 
     * @param name the name of the parameter bundle in network tables
     * @param clazz the class to use to get parameters
     */
    public ParameterBundle(String name, Class<T> clazz) {
        this(name, ParameterBundle.class.getResourceAsStream("/parameters/" + name + ".param"), clazz);
    }

    /**
     * Creates a parameter bundle that loads its defaults from the specified
     * file.
     * 
     * @param name the name of the bundle
     * @param file the file to load defaults from
     * @param clazz the class to use to get parameters
     * @throws FileNotFoundException
     */
    public ParameterBundle(String name, File file, Class<T> clazz) throws FileNotFoundException {
        this(name, ((Function<File, FileInputStream>) (t) -> {
            try {
                return new FileInputStream(file);
            } catch (Exception e) {
                System.err.println("Could not find parameter file: " + file.getAbsolutePath());
                return null;
            }
        }).apply(file), clazz);
    }

    /**
     * Creates a parameter bundle that loads its defaults from the input stream.
     * 
     * @param name the name of the bundle
     * @param storedParameters the input stream to load defaults from
     * @param clazz the class to use to get parameters
     * @throws FileNotFoundException
     */
    public ParameterBundle(String name, InputStream storedParameters, Class<T> clazz) {
        this(name, ((Function<InputStream, Properties>) (t) -> {
            Properties p = new Properties();
            if (t != null) {
                try {
                    p.load(t);
                } catch (IOException e) {
                    System.err.println("Could not load parameters from input stream: " + e);
                } finally {
                    try {
                        t.close();
                    } catch (Exception e) {
                    }
                }
            }
            return p;
        }).apply(storedParameters), clazz);
    }

    /**
     * Creates a parameter bundle that loads its defaults from an already
     * initialized {@link Properties} object.
     * 
     * @param name the name of the bundle
     * @param storedParameters the {@link Properties} to load defaults from
     * @param clazz the class to use to get parameters
     * @throws FileNotFoundException
     */
    public ParameterBundle(String name, Properties storedParameters, Class<T> clazz) {
        // Initialize parameter subtable
        table = PARAMETER_TABLE.getSubTable(name);

        // Load parameter list (possibly from cache)
        Parameter[] parameters = parameterCache.get(clazz);

        if (parameters == null) {
            parameters = clazz.getAnnotationsByType(Parameter.class);
            parameterCache.put(clazz, parameters);
        }
        this.parameters = new HashMap<>(
                Arrays.stream(parameters).collect(Collectors.toMap(Parameter::key, parameter -> parameter)));
        // Loop through all parameters
        for (Parameter p : parameters) {
            String key = p.key();

            // Try to load the value from the table
            Object value = table.getValue(key, null);
            if (value != null) {
                if (!value.getClass().equals(p.type().type)) {
                    table.delete(key);
                    value = null;
                } else {
                    // WARNING: THIS LINE IS TRICKY
                    // If the parameter already exists in the table, break out
                    // of this iteration of the loop
                    continue;
                }
            }

            // =================================================================
            // Nothing below here is executed if the parameter already exists in
            // the table
            // =================================================================

            // Fall back to the value from the properties file
            if (value == null) {
                String storedValueString = storedParameters.getProperty(key);
                if (storedValueString != null) {
                    switch (p.type()) {
                    case STRING:
                        value = storedValueString;
                    break;
                    case NUMBER:
                        try {
                            value = Double.parseDouble(storedValueString);
                        } catch (NumberFormatException ex) {
                            System.err.println("Could not parse parameter from file: " + key);
                        }
                    break;
                    case BOOLEAN:
                        value = Boolean.valueOf(storedValueString);
                    break;
                    }
                }
            }

            // Lastly, fall back to the defaults
            if (value == null) {
                switch (p.type()) {
                case STRING:
                    value = p.stringValue();
                break;
                case NUMBER:
                    value = p.numberValue();
                break;
                case BOOLEAN:
                    value = p.booleanValue();
                break;
                }
            }

            table.putValue(key, value);
        }
    }

    /**
     * Set a string parameter.
     * 
     * @param key the parameter name
     * @param value the parameter value
     */
    public void setString(String key, String value) {
        Parameter p = parameters.get(key);
        if (p != null) {
            table.putString(key, value);
        } else {
            throw new ParameterNotFoundException(key);
        }
    }

    /**
     * Set a number parameter.
     * 
     * @param key the parameter name
     * @param value the parameter value
     */
    public void setNumber(String key, double value) {
        Parameter p = parameters.get(key);
        if (p != null) {
            table.putNumber(key, value);
        } else {
            throw new ParameterNotFoundException(key);
        }
    }

    /**
     * Set a boolean parameter.
     * 
     * @param key the parameter name
     * @param value the parameter value
     */
    public void setBoolean(String key, boolean value) {
        Parameter p = parameters.get(key);
        if (p != null) {
            table.putBoolean(key, value);
        } else {
            throw new ParameterNotFoundException(key);
        }
    }

    /**
     * Set a string parameter.
     * 
     * @return the parameter value
     */
    public String getString(String key) {
        Parameter p = parameters.get(key);
        if (p != null) {
            return table.getString(key, p.stringValue());
        } else {
            throw new ParameterNotFoundException(key);
        }
    }

    /**
     * Set a number parameter.
     * 
     * @return the parameter value
     */
    public double getNumber(String key) {
        Parameter p = parameters.get(key);
        if (p != null) {
            return table.getNumber(key, p.numberValue());
        } else {
            throw new ParameterNotFoundException(key);
        }
    }

    /**
     * Set a boolean parameter.
     * 
     * @return the parameter value
     */
    public boolean getBoolean(String key) {
        Parameter p = parameters.get(key);
        if (p != null) {
            return table.getBoolean(key, p.booleanValue());
        } else {
            throw new ParameterNotFoundException(key);
        }
    }

    /**
     * Adds a listener that is notified when any parameter is changed. It is
     * also immediately called with the current values of all the parameters.
     * 
     * @param listener the listener to add
     */
    public void addListener(Listener listener) {
        table.addTableListener((table, key, val, isNew) -> {
            Parameter p = parameters.get(key);
            if (p != null && val.getClass().equals(p.type().type)) {
                listener.parameterChanged(key, p.type(), val);
            }
        }, true);
    }

    /**
     * Adds a listener that is notified when the specified parameter is changed.
     * It is also immediately called with the current value of the parameter.
     * 
     * @param listener the listener to add
     * @param key the name of the parameter
     */
    public void addListener(String key, Listener listener) {
        Parameter p = parameters.get(key);
        if (p != null) {
            table.addTableListener(key, (table, ntKey, val, isNew) -> {
                if (val.getClass().equals(p.type().type)) {
                    listener.parameterChanged(ntKey, p.type(), val);
                }
            }, true);
        } else {
            throw new ParameterNotFoundException(key);
        }
    }

    /**
     * @return the parameter network table
     */
    public ITable getTable() {
        return table;
    }
}
