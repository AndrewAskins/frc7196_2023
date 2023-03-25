package frc.robot;

public class CANLight {
    /**
     * @return The version of this library in the format <code>major.minor
     * </code>, for example: "1.1"
     */
    public static String getLibraryVersion() {
        return CANLightJNI.GetLibraryVersion();
    }
    
	private final int m_handle;
	
    /**
     * An instance of this object represents a single CANLight device. Multiple
     * devices can be used indepentently to control multiple light strips. Only
     * a single instance can be created for each device ID. Please construct
     * this object only once when initializing your robot and pass the reference
     * around. If you do wish to call this constructor with the same ID later,
     * call {@link #free} first.
     * <p>
     * If a CANLight does not have a CAN connection to a roboRIO, its default
     * behavior of {@link #cycle(int, int) cycle(1,7)} will be used. If it does
     * find a CAN connection, it will execute {@link #showRegister(int)
     * showRegister(0)}. The CANLight will continue to execute its last command
     * until a new one is issued via one of the methods detailed below.
     * <p>
     * The CANLight can hold a sequence of up to eight colors and associated
     * durations. Each register has a default value. The WriteRegister command
     * can be used to change these. The CANLight will restore its default values
     * when power is lost.
     * 
     * @param deviceNumber An integer between 1 and 60 (inclusive) for the ID of
     * this CANLight. CAN IDs can be modified through the mindsensors
     * configuration tool, available at
     * <a href="http://www.mindsensors.com/pages/311">mindsensors.com/pages/311
     * </a>. Devices will ship with a factory default CAN ID of 3. Please use a
     * unique ID for each device.
     */
	public CANLight(int deviceNumber) {
        if (deviceNumber > 60 || deviceNumber < 1)
            throw new IndexOutOfBoundsException("Device number must be between 1 and 60.");
		m_handle = CANLightJNI.Constructor(deviceNumber);
	}
    public void free() {
		CANLightJNI.Destructor(m_handle);
	}
	
    /**
     * @return The device ID provided when constructing this CANLight instance.
     */
    public int getDeviceID() {
		return CANLightJNI.GetDeviceID(m_handle);
	}
    /**
     * @return The name associated with this CANLight. The factory default will
     * be "CANLight", but this value can be changed through the mindsensors 
     * configuration tool.
     */
	public String getDeviceName() {
		return CANLightJNI.GetDeviceName(m_handle);
	}
    /**
     * @return The firmware version of this CANLight. Firmware can be updated
     * through the mindsensors configuration tool. This may be considered during
     * inspection at competitions. Firmware updates can provide new features.
     * The firmware version of the CANLight device must be compatible with this
     * library.
     */
	public String getFirmwareVersion() {
		return CANLightJNI.GetFirmwareVersion(m_handle);
	}
    /**
     * @return The hardware version of this CANLight. Any hardware revisions
     * will have a different hardware version number.
     */
	public String getHardwareVersion() {
		return CANLightJNI.GetHardwareVersion(m_handle);
	}
    /**
     * @return The bootloader version of this CANLight. The bootloader is used
     * to update firmware on the CANLight.
     */
	public String getBootloaderVersion() {
		return CANLightJNI.GetBootloaderVersion(m_handle);
	}
    /**
     * @return The serial number of this device. Each serial number is unique
     * and may be requested for customer support.
     */
	public String getSerialNumber() {
		return CANLightJNI.GetSerialNumber(m_handle);
	}
    
    /**
     * Each CANLight has a build-in LED on the board itself. This command will
     * cause it to blink for a specified duration. This can be useful in
     * debugging. Please do not confuse this with a fast flashing pattern, which
     * signifies that the CANLight can not find a connection to the FRC driver
     * station.
     * 
     * @param seconds The number of seconds to blink.
     */
	public void blinkLED(int seconds) {
        if (seconds <= 0)
            throw new IllegalArgumentException("Seconds must be a positive integer.");
        if (seconds > 255) seconds = 255;
		CANLightJNI.BlinkLED(m_handle, seconds);
	}
	
    /**
     * Set a static color for the CANLight to display. This command will simply
     * set red, green, and blue values for the RGB LED strip. The CANLight will
     * continue to display this color until a new command is called.
     * 
     * @param red An integer between 0 and 255 (inclusive) for the red component
     * of the color to show.
     * @param green An integer between 0 and 255 (inclusive) for the green
     * component of the color to show.
     * @param blue An integer between 0 and 255 (inclusive) for the blue
     * component of the color to show.
     */
	public void showRGB(int red, int green, int blue) {
        if (red > 255) red = 255;
        if (green > 255) green = 255;
        if (blue > 255) blue = 255;
        if (red < 0) red = 0;
        if (green < 0) green = 0;
        if (blue < 0) blue = 0;
        
		CANLightJNI.ShowRGB(m_handle, red, green, blue);
	}
    
    /**
     * Write a value in the CANLight's internal memory. The CANLight has 8
     * internal memory slots (registers) for use in commands like
     * {@link #cycle(int, int)}. The time value will determine how long a color
     * will display with {@link #flash(int)} or {@link #cycle(int, int)}, or how
     * long it will take to {@link #fade(int, int) fade} from this color. They
     * have preset values, but this command allows for changing the stored
     * colors. The registers will return to their default values when the
     * CANLight loses power. A robot's initialization function can be a good
     * place to set up these values.
     * 
     * @param index An integer between 0 and 7 (inclusive) for which register to
     * write to.
     * @param time The duration, in seconds, to use in commands like
     * {@link #flash(int)} or {@link #cycle(int, int)}. Value less than 1 can be
     * used, such as 0.25 for a quarter of a second.
     * @param red An integer between 0 and 255 (inclusive).
     * @param green An integer between 0 and 255 (inclusive).
     * @param blue An integer between 0 and 255 (inclusive).
     */
	public void writeRegister(int index, double time, int red, int green, int blue) {
        if (index > 7 || index < 0)
            throw new IndexOutOfBoundsException("Index must be between 0 and 7.");
        if (time < 0)
            throw new IllegalArgumentException("Time/duration must be positive.");
        int centiseconds = (int)Math.round(time*1000/10); // multiply by 1000 for milliseconds, divide by 10 for increment size
        if (centiseconds > 255) centiseconds = 255;
        if (red > 255) red = 255;
        if (green > 255) green = 255;
        if (blue > 255) blue = 255;
        if (red < 0) red = 0;
        if (green < 0) green = 0;
        if (blue < 0) blue = 0;
        
		CANLightJNI.WriteRegister(m_handle, index, centiseconds, red, green, blue);
	}
    
    /**
     * Restore the registers to power on default. These are, in order, from
     * index 0 to 7: off, red, green, blue, orange, teal, purple, white.
     */
	public void reset() {
        CANLightJNI.Reset(m_handle);
	}
    
    /**
     * Display a stored color. As with {@link #showRGB(int, int, int)} this
     * color will be displayed until a new command is issued.
     * 
     * @param index An integer between 0 and 7 (inclusive) for which register to
     * show.
     */
	public void showRegister(int index) {
        if (index > 7 || index < 0)
            throw new IndexOutOfBoundsException("Index must be between 0 and 7.");
		CANLightJNI.ShowRegister(m_handle, index);
	}
    
    /**
     * Flash a stored color. Lights will remain on and off for the time
     * specified in this register.
     * 
     * @param index An integer between 0 and 7 (inclusive) for which register to
     * show.
     */
	public void flash(int index) {
        if (index > 7 || index < 0)
            throw new IndexOutOfBoundsException("Index must be between 0 and 7.");
		CANLightJNI.Flash(m_handle, index);
	}
    
    /**
     * Cycle through a sequence of stored color values.
     * 
     * @param fromIndex An integer between 0 and 7 (inclusive) for which
     * register to begin the sequence at.
     * @param toIndex An integer between 0 and 7 (inclusive) for which register
     * to use as the last color in the sequence.
     */
	public void cycle(int fromIndex, int toIndex) {
        if (fromIndex > 7 || fromIndex < 0 || toIndex > 7 || toIndex < 0)
            throw new IndexOutOfBoundsException("Indices must be between 0 and 7.");
        if (fromIndex > toIndex) { // swap
            int temp = fromIndex;
            fromIndex = toIndex;
            toIndex = temp;
        }
		CANLightJNI.Cycle(m_handle, fromIndex, toIndex);
	}
    
    /**
     * Fade across a sequence of stored color values. Similar t.
     * {@link #cycle(int, int)}, but fading between colors instead of jumping to
     * them. The duration value of each register specifies how long it will take
     * to fade from that color.
     * 
     * @param startIndex An integer between 0 and 7 (inclusive) for which
     * register to begin at.
     * @param endIndex An integer between 0 and 7 (inclusive) for which register
     * to end at.
     */
	public void fade(int startIndex, int endIndex) {
        if (startIndex > 7 || startIndex < 0 || endIndex > 7 || endIndex < 0)
            throw new IndexOutOfBoundsException("Indices must be between 0 and 7.");
        if (startIndex > endIndex) { // swap
            int temp = startIndex;
            startIndex = endIndex;
            endIndex = temp;
        }
        CANLightJNI.Fade(m_handle, startIndex, endIndex);
	}
    
    /**
     * @return The voltage this CANLight device is currently receiving. A value
     * of 0.0 likely indicates this CANLight is not connected properly. Please
     * check the CAN and power connections, or look to the CANLight user guide
     * on mindsensors.com.
     */
    public double getBatteryVoltage() {
        return CANLightJNI.GetBatteryVoltage(m_handle);
    }
}
