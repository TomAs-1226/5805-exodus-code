package frc.robot.command;

/**
 * a result produced by {@link Command} 
 * reason for strange constructors is boredom
 * @author florpy
 * @see Command
 */
public final class CommandResult {

    /**
     * command result types
     * @author phalaena
     */
    public enum Type {
        /** indicates this command succeded in execution and doesnt need to be run again */
        SUCCESS,
        /** indicates this command failed in execution */
        FAILURE,
        /** indicates this command didnt fail execution, succeded at its current step, but is not done overall */
        INTEROP,
        /** indicates this command doesnt need to or can run now, and is giving right-of-way to the next command */
        YIELDED
    }

    /** result type */
    private final Type RESULT_TYPE;
    /** the log stored in the result */
    private final String LOG;

    /**
     * constructs a new command result with the specified type and debug log
     * @param resultType the type of the result specified by {@link Type}
     * @param debugLog the log of the result (any string)
     */
    //TODO: this maybe should be public? or protected
    private CommandResult(Type resultType, String debugLog) {
        this.RESULT_TYPE = resultType;
        this.LOG = debugLog;
    }

    /**
     * returns a successful command result
     * @param log log of what happened
     * @return successful command result w/ log
     */
    public static CommandResult success(String log) {
        return new CommandResult(Type.SUCCESS, log);
    }

    /**
     * returns a failed command result
     * @param log log of what happened
     * @return failed command result w/ log
     */
    public static CommandResult fail(String log) {
        return new CommandResult(Type.FAILURE, log);
    }

    /**
     * returns an interop command result
     * @param log log of what happened
     * @return interop command result w/ log
     */
    public static CommandResult interop(String log) {
        return new CommandResult(Type.INTEROP, log);
    }

    /**
     * returns a yielding command result
     * @param log log of what happened
     * @return yield command result w/ log
     */
    public static CommandResult yield(String log) {
        return new CommandResult(Type.YIELDED, log);
    }

    /**
     * returns the result's type
     * @return result type
     * @see Type
     */
    public Type getType() {
        return this.RESULT_TYPE;
    }

    /**
     * returns the string debug log attached to this command result
     * @return debug log string
     */
    public String getLog() {
        return this.LOG;
    }

}
