package org.team199.lib;

import java.util.Arrays;
import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;

import org.mockito.Mockito;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Checks for errors with motor controllers
 */
public final class MotorErrors {

    private static final HashMap<CANSparkMax, Short> flags = new HashMap<>();
    private static final HashMap<CANSparkMax, Short> stickyFlags = new HashMap<>();

    /**
     * Checks for errors when preforming a function on a phoenix motor
     * @param error The {@link ErrorCode} returned by the function
     * @see #reportError(CANError)
     * @see #reportErrors(ErrorCode...)
     * @see #reportErrors(CANError...)
     */
    public static void reportError(ErrorCode error) {
        reportError("CTRE", error, ErrorCode.OK);
    }

    /**
     * Checks for errors when preforming a function on a REV Robotics motor
     * @param error The {@link CANError} returned by the function
     * @see #reportError(ErrorCode)
     * @see #reportErrors(ErrorCode...)
     * @see #reportErrors(CANError...)
     */
    public static void reportError(CANError error) {
        reportError("REV Robotics", error, CANError.kOk);
    }
    
    /**
     * Checks for errors when preforming multiple functions on a phoenix motor
     * @param error The {@link ErrorCode}s returned by the functions
     * @see #reportError(ErrorCode)
     * @see #reportError(CANError)
     * @see #reportErrors(CANError...)
     */
    public static void reportErrors(ErrorCode... errors) {
        for(ErrorCode error: errors) {
            reportError(error);
        }
    }

    /**
     * Checks for errors when preforming multiple functions on a REV Robotics motor
     * @param error The {@link CANError}s returned by the functions
     * @see #reportError(ErrorCode)
     * @see #reportError(CANError)
     * @see #reportErrors(ErrorCode...)
     */
    public static void reportErrors(CANError... errors) {
        for(CANError error: errors) {
            reportError(error);
        }
    }

    /**
     * Checks for an error when preforming a function on a motor controller
     * @param <T> The {@link Enum} used by the vendor of the motor controller to report errors
     * @param vendor The vendor of the motor controller used for debug messages
     * @param error The error code returned by the motor controller
     * @param ok The ok value of the error to ignore or <code>null</code> if none exists
     */
    private static <T extends Enum<T>> void reportError(String vendor, T error, T ok) {
        if(error == null || error == ok) {
            return;
        }
        DriverStation.reportError("Error: " + error.name() + " occured while configuring " + vendor + " motor", true);
    }

    /**
     * Checks for spark max faults on a {@link CANSparkMax} and adds it to be checked when {@link #printSparkMaxErrorMessages()} is called
     * @param spark The {@link CANSparkMax} to check
     * @see #printSparkMaxErrorMessages()
     */
    public static void checkSparkMaxErrors(CANSparkMax spark) {     
        //Purposely obivously impersonal to differentiate from actual computer generated errors
        short faults = spark.getFaults();
        short stickyFaults = spark.getStickyFaults();
        short prevFaults = flags.containsKey(spark) ? flags.get(spark) : 0;
        short prevStickyFaults = stickyFlags.containsKey(spark) ? stickyFlags.get(spark) : 0;

        if (spark.getFaults() != 0 && prevFaults != faults) {
        DriverStation.reportError("Whoops, big oopsie : fault error(s) with spark max id : " + spark.getDeviceId() + ": [ " + formatFaults(spark) + "], ooF!", false);
        }
        if (spark.getStickyFaults() != 0 && prevStickyFaults != stickyFaults) {
        DriverStation.reportError("Bruh, you did an Error : sticky fault(s) error with spark max id : " + spark.getDeviceId() + ": " + formatStickyFaults(spark) + ", Ouch!", false);
        }
        spark.clearFaults();
        flags.put(spark, faults);
        stickyFlags.put(spark, stickyFaults);
    }

    private static String formatFaults(CANSparkMax spark) {
        String out = "";
        for(FaultID fault: FaultID.values()) {
            if(spark.getFault(fault)) {
                out += (fault.name() + " ");
            }
        }
        return out;
    }

    private static String formatStickyFaults(CANSparkMax spark) {
        String out = "";
        for(FaultID fault: FaultID.values()) {
            if(spark.getStickyFault(fault)) {
                out += (fault.name() + " ");
            }
        }
        return out;
    }

    /**
     * Calls {@link #checkSparkMaxErrors(CANSparkMax)} on all {@link CANSparkMax}s for which that method has previously been called on
     * @see {@link #checkSparkMaxErrors(CANSparkMax)}
     */
    public static void printSparkMaxErrorMessages() {
        flags.keySet().forEach((spark) -> checkSparkMaxErrors(spark));
    }

    /**
     * Utilizes {@link Mockito} to create a {@link CANSparkMax} with no funtionality
     * @return A {@link CANSparkMax} with no funtionality
     * @see DummySparkMaxAnswer
     */
    public static CANSparkMax createDummySparkMax() {
        return Mockito.mock(CANSparkMax.class, new DummySparkMaxAnswer());
    }

    private MotorErrors() {}

}