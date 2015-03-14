/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * A {@link Command} that has parameters which can be set interactively through
 * the SmartDashboard.
 * 
 * @author Ben Wolsieffer
 */
public abstract class ParameterCommand extends Command {

    private ITable parameterTable;
    private final HashMap<String, Object> defaultParameterValues = new HashMap<>(1);

    public ParameterCommand() {
    }

    public ParameterCommand(double timeout) {
        super(timeout);
    }

    /**
     * Adds a new number parameter to the command.
     * 
     * @param key the name of the parameter
     * @param defaultValue the default value of the parameter
     */
    public void addNumberParameter(String key, double defaultValue) {
        if (parameterTable != null) {
            parameterTable.putNumber(key, defaultValue);
        }
        defaultParameterValues.put(key, defaultValue);
    }

    /**
     * Gets the value of a number parameter.
     * 
     * @param key the name of the parameter
     * @return the value or default value of the parameter
     * @throws IllegalArgumentException if the parameter does not exist or it is
     *             not a number
     */
    public double getNumberParameter(String key) {
        Object defaultObject = defaultParameterValues.get(key);
        if (defaultObject == null || !(defaultObject instanceof Number)) {
            throw new IllegalArgumentException("Parameter \"" + key + "\" does not exist or is not a number");
        }
        if (parameterTable != null) {
            return parameterTable.getNumber(key, (double) defaultObject);
        } else {
            return (double) defaultObject;
        }
    }

    /**
     * Adds a new string parameter to the command.
     * 
     * @param key the name of the parameter
     * @param defaultValue the default value of the parameter
     */
    public void addStringParameter(String key, String defaultValue) {
        if (parameterTable != null) {
            parameterTable.putString(key, defaultValue);
        }
        defaultParameterValues.put(key, defaultValue);
    }

    /**
     * Gets the value of a {@link String} parameter.
     * 
     * @param key the name of the parameter
     * @return the value or default value of the parameter
     * @throws IllegalArgumentException if the parameter does not exist or it is
     *             not a {@link String}
     */
    public String getStringParameter(String key) {
        Object defaultObject = defaultParameterValues.get(key);
        if (defaultObject == null || !(defaultObject instanceof String)) {
            throw new IllegalArgumentException("Parameter \"" + key + "\" does not exist or is not a String");
        }
        if (parameterTable != null) {
            return parameterTable.getString(key, (String) defaultObject);
        } else {
            return (String) defaultObject;
        }
    }

    /**
     * Adds a new boolean parameter to the command.
     * 
     * @param key the name of the parameter
     * @param defaultValue the default value of the parameter
     */
    public void addBooleanParameter(String key, boolean defaultValue) {
        if (parameterTable != null) {
            parameterTable.putBoolean(key, defaultValue);
        }
        defaultParameterValues.put(key, defaultValue);
    }

    /**
     * Gets the value of a boolean parameter.
     * 
     * @param key the name of the parameter
     * @return the value or default value of the parameter
     * @throws IllegalArgumentException if the parameter does not exist or it is
     *             not a boolean
     */
    public boolean getBooleanParameter(String key) {
        Object defaultObject = defaultParameterValues.get(key);
        if (defaultObject == null || !(defaultObject instanceof Boolean)) {
            throw new IllegalArgumentException("Parameter \"" + key + "\" does not exist or is not a boolean");
        }
        if (parameterTable != null) {
            return parameterTable.getBoolean(key, (boolean) defaultObject);
        } else {
            return (boolean) defaultObject;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getSmartDashboardType() {
        return "Parameter Command";
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void initTable(ITable table) {
        super.initTable(table);
        if (table != null) {
            // Create a parameter subtable
            parameterTable = table.getSubTable("Parameters");
            // Put default values in table
            defaultParameterValues.forEach(parameterTable::putValue);
        } else {
            // If the table does not exist, neither do the parameters
            parameterTable = null;
        }
    }
}
