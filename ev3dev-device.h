/*
 * ev3dev-device.h

 *
 *  Criado em: 17/11/2018
 *  Author: Julio Cesar Goldner Vendramini
 */
//Classes de apoio para criação das classes específicas de cada sensor e motor
#ifndef EV3DEV_DEVICE_H_
#define EV3DEV_DEVICE_H_


//-----------------------------------------------------------------------------

typedef std::string         tipoDispositivo;
typedef std::string         tipoModo;
typedef std::set<tipoModo> mode_set;
typedef std::string         enderecoPorta;

//-----------------------------------------------------------------------------

const enderecoPorta INPUT_AUTO;  //!< Automatic input selection
const enderecoPorta OUTPUT_AUTO; //!< Automatic output selection

/*Esta versão da lib só aceita o EV3*/
constexpr char ENTRADA_1[]  = "ev3-ports:in1";  //!< Sensor port 1
constexpr char ENTRADA_2[]  = "ev3-ports:in2";  //!< Sensor port 2
constexpr char ENTRADA_3[]  = "ev3-ports:in3";  //!< Sensor port 3
constexpr char ENTRADA_4[]  = "ev3-ports:in4";  //!< Sensor port 4

constexpr char SAIDA_A[] = "ev3-ports:outA"; //!< Motor port A
constexpr char SAIDA_B[] = "ev3-ports:outB"; //!< Motor port B
constexpr char SAIDA_C[] = "ev3-ports:outC"; //!< Motor port C
constexpr char SAIDA_D[] = "ev3-ports:outD"; //!< Motor port D

//-----------------------------------------------------------------------------



class device
{
public:
	bool connect(const std::string &dir,
			const std::string &pattern,
			const std::map<std::string, std::set<std::string>> &match) noexcept;

	inline bool connected() const { return !path.empty(); }

	int         device_index() const;

	int         get_attr_int   (const std::string &name) const;
	void        set_attr_int   (const std::string &name,
			int value);
	std::string get_attr_string(const std::string &name) const;
	void        set_attr_string(const std::string &name,
			const std::string &value);

	std::string get_attr_line  (const std::string &name) const;
	mode_set    get_attr_set   (const std::string &name,
			std::string *pCur = nullptr) const;

	std::string get_attr_from_set(const std::string &name) const;

protected:
	std::string path;
	mutable int _device_index = -1;
	char port; //porta que o sensor está conectado
};
/**
//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.sensor>currentClass

// The sensor class provides a uniform interface for using most of the
// sensors available for the EV3. The various underlying device drivers will
// create a `lego-sensor` device for interacting with the sensors.
//
// Sensors are primarily controlled by setting the `mode` and monitored by
// reading the `value<N>` attributes. Values can be converted to floating point
// if needed by `value<N>` / 10.0 ^ `decimals`.
//
// Since the name of the `sensor<N>` device node does not correspond to the port
// that a sensor is plugged in to, you must look at the `address` attribute if
// you need to know which port a sensor is plugged in to. However, if you don't
// have more than one sensor of each type, you can just look for a matching
// `driver_name`. Then it will not matter which port a sensor is plugged in to - your
// program will still work.

//~autogen
 *
 */
class sensor : protected device
{
public:
	typedef tipoDispositivo tipoSensor;

	static constexpr char ev3_touch[]      = "lego-ev3-touch";
	static constexpr char ev3_color[]      = "lego-ev3-color";
	static constexpr char ev3_ultrasonic[] = "lego-ev3-us";
	static constexpr char ev3_gyro[]       = "lego-ev3-gyro";
	static constexpr char ev3_infrared[]   = "lego-ev3-ir";

	sensor(enderecoPorta);
	sensor(enderecoPorta, const std::set<tipoSensor>&);

	using device::connected;
	using device::device_index;

	// Returns the value or values measured by the sensor. Check `num_values` to
	// see how many values there are. Values with index >= num_values will return
	// an error. The values are fixed point numbers, so check `decimals` to see
	// if you need to divide to get the actual value.
	int   value(unsigned index=0) const;

	// The value converted to float using `decimals`.
	float float_value(unsigned index=0) const;

	// Human-readable name of the connected sensor.
	std::string type_name() const;

	// Bin Data Format: read-only
	// Returns the format of the values in `bin_data` for the current mode.
	// Possible values are:
	//
	//    - `u8`: Unsigned 8-bit integer (byte)
	//    - `s8`: Signed 8-bit integer (sbyte)
	//    - `u16`: Unsigned 16-bit integer (ushort)
	//    - `s16`: Signed 16-bit integer (short)
	//    - `s16_be`: Signed 16-bit integer, big endian
	//    - `s32`: Signed 32-bit integer (int)
	//    - `float`: IEEE 754 32-bit floating point (float)
	std::string bin_data_format() const { return get_attr_string("bin_data_format"); };

	// Bin Data: read-only
	// Returns the unscaled raw values in the `value<N>` attributes as raw byte
	// array. Use `bin_data_format`, `num_values` and the individual sensor
	// documentation to determine how to interpret the data.
	const std::vector<char>& bin_data() const;

	// Bin Data: read-only
	// Writes the unscaled raw values in the `value<N>` attributes into the
	// user-provided struct/buffer.  Use `bin_data_format`, `num_values` and the
	// individual sensor documentation to determine how to interpret the data.
	template <class T>
	void bin_data(T *buf) const {
		bin_data(); // fills _bin_data
		std::copy_n(_bin_data.data(), _bin_data.size(), reinterpret_cast<char*>(buf));
	}

	//~autogen generic-get-set classes.sensor>currentClass

	// Address: read-only
	// Returns the name of the port that the sensor is connected to, e.g. `ev3:in1`.
	// I2C sensors also include the I2C address (decimal), e.g. `ev3:in1:i2c8`.
	std::string address() const { return get_attr_string("address"); }

	// Command: write-only
	// Sends a command to the sensor.
	auto set_command(std::string v) -> decltype(*this) {
		set_attr_string("command", v);
		return *this;
	}

	// Commands: read-only
	// Returns a list of the valid commands for the sensor.
	// Returns -EOPNOTSUPP if no commands are supported.
	mode_set commands() const { return get_attr_set("commands"); }

	// Decimals: read-only
	// Returns the number of decimal places for the values in the `value<N>`
	// attributes of the current mode.
	int decimals() const { return get_attr_int("decimals"); }

	// Driver Name: read-only
	// Returns the name of the sensor device/driver. See the list of [supported
	// sensors] for a complete list of drivers.
	std::string driver_name() const { return get_attr_string("driver_name"); }

	// Mode: read/write
	// Returns the current mode. Writing one of the values returned by `modes`
	// sets the sensor to that mode.
	std::string mode() const { return get_attr_string("mode"); }
	auto setModo(std::string v) -> decltype(*this) {
		set_attr_string("mode", v);
		return *this;
	}

	// Modes: read-only
	// Returns a list of the valid modes for the sensor.
	mode_set modes() const { return get_attr_set("modes"); }

	// Num Values: read-only
	// Returns the number of `value<N>` attributes that will return a valid value
	// for the current mode.
	int num_values() const { return get_attr_int("num_values"); }

	// Units: read-only
	// Returns the units of the measured value for the current mode. May return
	// empty string
	std::string units() const { return get_attr_string("units"); }


	//~autogen

protected:
	sensor() {}

	bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;

	mutable std::vector<char> _bin_data;
};

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.legoPort>currentClass

// The `lego-port` class provides an interface for working with input and
// output ports that are compatible with LEGO MINDSTORMS RCX/NXT/EV3, LEGO
// WeDo and LEGO Power Functions sensors and motors. Supported devices include
// the LEGO MINDSTORMS EV3 Intelligent Brick, the LEGO WeDo USB hub and
// various sensor multiplexers from 3rd party manufacturers.
//
// Some types of ports may have multiple modes of operation. For example, the
// input ports on the EV3 brick can communicate with sensors using UART, I2C
// or analog validate signals - but not all at the same time. Therefore there
// are multiple modes available to connect to the different types of sensors.
//
// In most cases, ports are able to automatically detect what type of sensor
// or motor is connected. In some cases though, this must be manually specified
// using the `mode` and `set_device` attributes. The `mode` attribute affects
// how the port communicates with the connected device. For example the input
// ports on the EV3 brick can communicate using UART, I2C or analog voltages,
// but not all at the same time, so the mode must be set to the one that is
// appropriate for the connected sensor. The `set_device` attribute is used to
// specify the exact type of sensor that is connected. Note: the mode must be
// correctly set before setting the sensor type.
//
// Ports can be found at `/sys/class/lego-port/port<N>` where `<N>` is
// incremented each time a new port is registered. Note: The number is not
// related to the actual port at all - use the `address` attribute to find
// a specific port.

//~autogen
class lego_port : protected device
{
public:
	lego_port(enderecoPorta);

	using device::connected;
	using device::device_index;

	//~autogen generic-get-set classes.legoPort>currentClass

	// Address: read-only
	// Returns the name of the port. See individual driver documentation for
	// the name that will be returned.
	std::string address() const { return get_attr_string("address"); }

	// Driver Name: read-only
	// Returns the name of the driver that loaded this device. You can find the
	// complete list of drivers in the [list of port drivers].
	std::string driver_name() const { return get_attr_string("driver_name"); }

	// Modes: read-only
	// Returns a list of the available modes of the port.
	mode_set modes() const { return get_attr_set("modes"); }

	// Mode: read/write
	// Reading returns the currently selected mode. Writing sets the mode.
	// Generally speaking when the mode changes any sensor or motor devices
	// associated with the port will be removed new ones loaded, however this
	// this will depend on the individual driver implementing this class.
	std::string mode() const { return get_attr_string("mode"); }
	auto setModo(std::string v) -> decltype(*this) {
		set_attr_string("mode", v);
		return *this;
	}

	// Set Device: write-only
	// For modes that support it, writing the name of a driver will cause a new
	// device to be registered for that driver and attached to this port. For
	// example, since NXT/Analog sensors cannot be auto-detected, you must use
	// this attribute to load the correct driver. Returns -EOPNOTSUPP if setting a
	// device is not supported.
	auto set_set_device(std::string v) -> decltype(*this) {
		set_attr_string("set_device", v);
		return *this;
	}

	// Status: read-only
	// In most cases, reading status will return the same value as `mode`. In
	// cases where there is an `auto` mode additional values may be returned,
	// such as `no-device` or `error`. See individual port driver documentation
	// for the full list of possible values.
	std::string status() const { return get_attr_string("status"); }


	//~autogen

protected:
	lego_port() {}

	bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
};



#endif /* EV3DEV_DEVICE_H_ */
