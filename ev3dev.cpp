/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 *
 * Copyright (c) 2014 - Franz Detro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Modification:
 *  Add new button for ev3dev Release 02.00.00 (ev3dev-jessie-2014-07-12) - Christophe Chaudelet
 *
 */
/*Simplificada e modificada por Julio Cesar Goldner Vendramini
 */
#include "ev3dev.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <list>
#include <map>
#include <array>
#include <algorithm>
#include <system_error>
#include <mutex>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <string.h>
#include <math.h>

#include <dirent.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#ifndef SYS_ROOT
#define SYS_ROOT "/sys/class"
#endif

#ifndef FSTREAM_CACHE_SIZE
#define FSTREAM_CACHE_SIZE 16
#endif

#ifndef NO_LINUX_HEADERS
#include <linux/fb.h>
#include <linux/input.h>
#else
#define KEY_CNT 8
#endif
static const int bits_per_long = sizeof(long) * 8;

//-----------------------------------------------------------------------------

namespace ev3dev {

namespace {

// This class implements a small LRU cache. It assumes the number of elements
// is small, and so uses a simple linear search.
template <typename K, typename V>
class lru_cache
{
private:
	// typedef st::pair<K, V> item;
	// std::pair seems to be missing necessary move constructors :(
	struct item
	{
		K first;
		V second;

		item(const K &k) : first(k) {}
		item(item &&m) : first(std::move(m.first)), second(std::move(m.second)) {}
	};

public:
	lru_cache(size_t size = 3) : _size(size)
{
}

	V &operator[] (const K &k)
	{
		iterator i = find(k);
		if (i != _items.end())
		{
			// Found the key, bring the item to the front.
			_items.splice(_items.begin(), _items, i);
		} else {
			// If the cache is full, remove oldest items to make room.
			while (_items.size() + 1 > _size)
			{
				_items.pop_back();
			}
			// Insert a new default constructed value for this new key.
			_items.emplace_front(k);
		}
		// The new item is the most recently used.
		return _items.front().second;
	}

	void clear()
	{
		_items.clear();
	}

private:
	typedef typename std::list<item>::iterator iterator;

	iterator find(const K &k)
	{
		return std::find_if(_items.begin(), _items.end(),
				[&](const item &i) { return i.first == k; });
	}

	size_t _size;
	std::list<item> _items;
};

// A global cache of files.
std::ifstream& ifstream_cache(const std::string &path) {
	static lru_cache<std::string, std::ifstream> cache(FSTREAM_CACHE_SIZE);
	static std::mutex mx;

	std::lock_guard<std::mutex> lock(mx);
	return cache[path];
}

std::ofstream& ofstream_cache(const std::string &path) {
	static lru_cache<std::string, std::ofstream> cache(FSTREAM_CACHE_SIZE);
	static std::mutex mx;

	std::lock_guard<std::mutex> lock(mx);
	return cache[path];
}

//-----------------------------------------------------------------------------

std::ofstream &ofstream_open(const std::string &path)
{
	std::ofstream &file = ofstream_cache(path);
	if (!file.is_open())
	{
		// Don't buffer writes to avoid latency. Also saves a bit of memory.
		file.rdbuf()->pubsetbuf(NULL, 0);
		file.open(path);
	}
	else
	{
		// Clear the error bits in case something happened.
		file.clear();
	}
	return file;
}

std::ifstream &ifstream_open(const std::string &path)
{
	std::ifstream &file = ifstream_cache(path);
	if (!file.is_open())
	{
		file.open(path);
	}
	else
	{
		// Clear the flags bits in case something happened (like reaching EOF).
		file.clear();
		file.seekg(0, std::ios::beg);
	}
	return file;
}

} // namespace

//-----------------------------------------------------------------------------

bool device::connect(
		const std::string &dir,
		const std::string &pattern,
		const std::map<std::string, std::set<std::string>> &match
) noexcept
		{
	using namespace std;
	const size_t pattern_length = pattern.length();

	struct dirent *dp;
	DIR *dfd;
	if ((dfd = opendir(dir.c_str())) != nullptr)
	{
		while ((dp = readdir(dfd)) != nullptr)
		{
			if (strncmp(dp->d_name, pattern.c_str(), pattern_length)==0)
			{
				try
				{
					path = dir + dp->d_name + '/';
					bool bMatch = true;
					for (auto &m : match)
					{
						const auto &attribute = m.first;
						const auto &matches   = m.second;
						const auto strValue   = get_attr_string(attribute);

						if (!matches.empty() && !matches.begin()->empty() &&
								(matches.find(strValue) == matches.end()))
						{
							bMatch = false;
							break;
						}
					}

					if (bMatch) {
						closedir(dfd);
						return true;
					}
				}
				catch (...) { }

				path.clear();
			}
		}

		closedir(dfd);
	}

	return false;
		}

//-----------------------------------------------------------------------------

int device::device_index() const
{
	using namespace std;
	if (path.empty()){
		string erro = "O dispositivo na porta ";
		erro += port;
		erro += " não está conectado.";
		Som::falar(erro,true);
		throw system_error(make_error_code(errc::function_not_supported), erro);
	}
	if (_device_index < 0)
	{
		unsigned f = 1;
		_device_index = 0;
		for (auto it=path.rbegin(); it!=path.rend(); ++it)
		{
			if(*it =='/')
				continue;
			if ((*it < '0') || (*it > '9'))
				break;

			_device_index += (*it -'0') * f;
			f *= 10;
		}
	}

	return _device_index;
}

//-----------------------------------------------------------------------------

int device::get_attr_int(const std::string &name) const {
	using namespace std;

	if (path.empty()){
		string erro = "O dispositivo na porta ";
		erro += port;
		erro += " não está conectado.";
		Som::falar(erro,true);
		throw system_error(make_error_code(errc::function_not_supported), erro);
	}
	for(int attempt = 0; attempt < 2; ++attempt) {
		ifstream &is = ifstream_open(path + name);
		if (is.is_open())
		{
			int result = 0;
			try {
				is >> result;
				return result;
			} catch(...) {
				// This could mean the sysfs attribute was recreated and the
				// corresponding file handle got stale. Lets close the file and try
				// again (once):
				if (attempt != 0) throw;

				is.close();
				is.clear();
			}
		} else break;
	}
	string erro = "O dispositivo na porta ";
	erro += port;
	erro += " parou de funcionar! ";
	Som::falar(erro,true);
	erro += path+name;
	throw system_error(make_error_code(errc::no_such_device), erro);
}

//-----------------------------------------------------------------------------

void device::set_attr_int(const std::string &name, int value) {
	using namespace std;

	if (path.empty()){
		string erro = "O dispositivo na porta ";
		erro += port;
		erro += " não está conectado.";
		Som::falar(erro,true);
		throw system_error(make_error_code(errc::function_not_supported), erro);
	}
	for(int attempt = 0; attempt < 2; ++attempt) {
		ofstream &os = ofstream_open(path + name);
		if (os.is_open())
		{
			if (os << value) return;

			// An error could mean that sysfs attribute was recreated and the cached
			// file handle is stale. Lets close the file and try again (once):
			if (attempt == 0 && errno == ENODEV) {
				os.close();
				os.clear();
			} else {
				throw system_error(std::error_code(errno, std::system_category()));
			}
		} else {
			string erro = "O dispositivo na porta ";
			erro += port;
			erro += " parou de funcionar! ";
			Som::falar(erro,true);
			erro += path+name;
			throw system_error(make_error_code(errc::no_such_device), erro);
		}
	}
}

//-----------------------------------------------------------------------------

std::string device::get_attr_string(const std::string &name) const
{
	using namespace std;
	if (path.empty()){
		string erro = "O dispositivo na porta ";
		erro += port;
		erro += " não está conectado.";
		Som::falar(erro,true);
		throw system_error(make_error_code(errc::function_not_supported), erro);
	}
	ifstream &is = ifstream_open(path + name);
	if (is.is_open())
	{
		string result;
		is >> result;
		return result;
	}

	string erro = "O dispositivo na porta ";
	erro += port;
	erro += " parou de funcionar! ";
	Som::falar(erro,true);
	erro += path+name;
	throw system_error(make_error_code(errc::no_such_device), erro);
}

//-----------------------------------------------------------------------------
//implementado aviso sobre o problema de dispositivos
void device::set_attr_string(const std::string &name, const std::string &value)
{
	using namespace std;

	if (path.empty()){
		string erro = "O dispositivo na porta ";
		erro += port;
		erro += " não está conectado.";
		Som::falar(erro,true);
		throw system_error(make_error_code(errc::function_not_supported), erro);
	}
	ofstream &os = ofstream_open(path + name);
	if (os.is_open())
	{
		if (!(os << value)) throw system_error(std::error_code(errno, std::system_category()));
		return;
	}
	string erro = "O dispositivo na porta ";
	erro += port;
	erro += " parou de funcionar! ";
	Som::falar(erro,true);
	erro += path+name;
	throw system_error(make_error_code(errc::no_such_device), erro);
}

//-----------------------------------------------------------------------------

std::string device::get_attr_line(const std::string &name) const
{
	using namespace std;

	if (path.empty()){
		string erro = "O dispositivo na porta ";
		erro += port;
		erro += " não está conectado.";
		Som::falar(erro,true);
		throw system_error(make_error_code(errc::function_not_supported), erro);
	}
	ifstream &is = ifstream_open(path + name);
	if (is.is_open())
	{
		string result;
		getline(is, result);
		return result;
	}

	string erro = "O dispositivo na porta ";
	erro += port;
	erro += " parou de funcionar! ";
	Som::falar(erro,true);
	erro += path+name;
	throw system_error(make_error_code(errc::no_such_device), erro);
}

//-----------------------------------------------------------------------------

mode_set device::get_attr_set(const std::string &name,
		std::string *pCur) const
{
	using namespace std;

	string s = get_attr_line(name);

	mode_set result;
	size_t pos, last_pos = 0;
	string t;
	do {
		pos = s.find(' ', last_pos);

		if (pos != string::npos)
		{
			t = s.substr(last_pos, pos-last_pos);
			last_pos = pos+1;
		}
		else
			t = s.substr(last_pos);

		if (!t.empty())
		{
			if (*t.begin()=='[')
			{
				t = t.substr(1, t.length()-2);
				if (pCur)
					*pCur = t;
			}
			result.insert(t);
		}
	} while (pos!=string::npos);

	return result;
}

//-----------------------------------------------------------------------------

std::string device::get_attr_from_set(const std::string &name) const
{
	using namespace std;

	string s = get_attr_line(name);

	size_t pos, last_pos = 0;
	string t;
	do {
		pos = s.find(' ', last_pos);

		if (pos != string::npos)
		{
			t = s.substr(last_pos, pos-last_pos);
			last_pos = pos+1;
		}
		else
			t = s.substr(last_pos);

		if (!t.empty())
		{
			if (*t.begin()=='[')
			{
				return t.substr(1, t.length()-2);
			}
		}
	} while (pos!=string::npos);

	return { "none" };
}

//-----------------------------------------------------------------------------

constexpr char sensor::ev3_touch[];
constexpr char sensor::ev3_color[];
constexpr char sensor::ev3_ultrasonic[];
constexpr char sensor::ev3_gyro[];
constexpr char sensor::ev3_infrared[];

//-----------------------------------------------------------------------------

sensor::sensor(enderecoPorta address)
{
	connect({{ "address", { address }}});
	port = address.c_str()[12];//pega o numero da porta do EV3
}

//-----------------------------------------------------------------------------

sensor::sensor(enderecoPorta address, const std::set<tipoSensor> &types)
{
	connect({{ "address", { address }},
		{ "driver_name", types }});
	port = address.c_str()[12];//pega o numero da porta do EV3
}

//-----------------------------------------------------------------------------

bool sensor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
		{
	static const std::string _strClassDir { SYS_ROOT "/lego-sensor/" };
	static const std::string _strPattern  { "sensor" };

	try
	{
		if (device::connect(_strClassDir, _strPattern, match))
		{
			return true;
		}
	}
	catch (...) { }
	path.clear();

	return false;
		}

//-----------------------------------------------------------------------------

std::string sensor::type_name() const
{
	auto type = driver_name();
	if (type.empty())
	{
		static const std::string s("<none>");
		return s;
	}

	static const std::map<tipoSensor, const std::string> lookup_table {
		{ ev3_touch,       "EV3 touch" },
		{ ev3_color,       "EV3 color" },
		{ ev3_ultrasonic,  "EV3 ultrasonic" },
		{ ev3_gyro,        "EV3 gyro" },
		{ ev3_infrared,    "EV3 infrared" },
	};

	auto s = lookup_table.find(type);
	if (s != lookup_table.end())
		return s->second;

	return type;
}

//-----------------------------------------------------------------------------

int sensor::value(unsigned index) const
{
	if (static_cast<int>(index) >= num_values())
		throw std::invalid_argument("index");

	char svalue[7] = "value0";
	svalue[5] += index;

	return get_attr_int(svalue);
}

//-----------------------------------------------------------------------------

float sensor::float_value(unsigned index) const
{
	return value(index) * powf(10, -decimals());
}

//-----------------------------------------------------------------------------
const std::vector<char>& sensor::bin_data() const
{
	using namespace std;

	if (path.empty())
		throw system_error(make_error_code(errc::function_not_supported), "no device connected");

	if (_bin_data.empty()) {
		static const map<string, int> lookup_table {
			{"u8",     1},
			{"s8",     1},
			{"u16",    2},
			{"s16",    2},
			{"s16_be", 2},
			{"s32",    4},
			{"float",  4}
		};

		int value_size = 1;

		auto s = lookup_table.find(bin_data_format());
		if (s != lookup_table.end())
			value_size = s->second;

		_bin_data.resize(num_values() * value_size);
	}

	const string fname = path + "bin_data";
	ifstream &is = ifstream_open(fname);
	if (is.is_open())
	{
		is.read(_bin_data.data(), _bin_data.size());
		return _bin_data;
	}

	throw system_error(make_error_code(errc::no_such_device), fname);
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.colorSensor>currentClass

constexpr char SensorCor::modo_col_reflect[];
constexpr char SensorCor::modo_col_ambient[];
constexpr char SensorCor::modo_col_color[];
constexpr char SensorCor::modo_ref_raw[];
constexpr char SensorCor::modo_rgb_raw[];
constexpr char SensorCor::corSemCor[];
constexpr char SensorCor::corPreta[];
constexpr char SensorCor::corAzul[];
constexpr char SensorCor::corVerde[];
constexpr char SensorCor::corAmarela[];
constexpr char SensorCor::corVermelha[];
constexpr char SensorCor::corBranca[];
constexpr char SensorCor::corMarrom[];

//~autogen

SensorCor::SensorCor(enderecoPorta portaSensor) :
		  sensor(portaSensor, { ev3_color })
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.ultrasonicSensor>currentClass

constexpr char SensorUltrassonico::modo_cont_dist_cm[];
constexpr char SensorUltrassonico::modo_ouvindo[];
constexpr char SensorUltrassonico::modo_unico_dist_cm[];

//~autogen

SensorUltrassonico::SensorUltrassonico(enderecoPorta address) :
		  sensor(address, { ev3_ultrasonic})
{
}

SensorUltrassonico::SensorUltrassonico(enderecoPorta address, const std::set<tipoSensor>& sensorTypes) :
		  sensor(address, sensorTypes)
{
}

constexpr char SensorGiroscopio::modoAngulo[];
constexpr char SensorGiroscopio::modoRotacaoPorSegundo[];
//constexpr char SensorGiroscopio::mode_gyro_fas[];
constexpr char SensorGiroscopio::modoAnguloERotacaoPorSegundo[];
//constexpr char SensorGiroscopio::mode_gyro_cal[];

SensorGiroscopio::SensorGiroscopio(enderecoPorta portaSensor) :
		  sensor(portaSensor, { ev3_gyro })
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.infraredSensor>currentClass

constexpr char SensorInfravermelho::modo_proximidade[];
constexpr char SensorInfravermelho::modo_ir_seek[];
constexpr char SensorInfravermelho::modo_ir_cal[];

//~autogen

SensorInfravermelho::SensorInfravermelho(enderecoPorta portaSensor) :
		  sensor(portaSensor, { ev3_infrared })
{
}


//-----------------------------------------------------------------------------

constexpr char Motor::motorGrande[];
constexpr char Motor::motorMedio[];

//~autogen generic-define-property-value classes.motor>currentClass

constexpr char Motor::comandoGiraParaSempre[];
constexpr char Motor::comandoGiraParaPosicaoAbsoluta[];
constexpr char Motor::comandoGiraParaPosicaoRelativa[];
constexpr char Motor::comandoGiraPorTempoDeterminado[];
constexpr char Motor::comandoGiraDireto[];
constexpr char Motor::comandoParar[];
constexpr char Motor::comandoReset[];
constexpr char Motor::polaridadeEncoderNormal[];
constexpr char Motor::polaridadeEncoderInvertida[];
constexpr char Motor::polaridadeNormal[];
constexpr char Motor::polaridadeInvertida[];
constexpr char Motor::estadoGirando[];
constexpr char Motor::estadoAcelerando[];
constexpr char Motor::estadoMantendoPosicao[];
constexpr char Motor::estadoSobrecarregado[];
constexpr char Motor::estadoTravado[];
constexpr char Motor::acaoParadaLivre[];
constexpr char Motor::acaoParadaFreio[];
constexpr char Motor::acaoParadaTravar[];

//~autogen

//-----------------------------------------------------------------------------

Motor::Motor(enderecoPorta address)
{
	connect({{ "address", { address } }});
	port = address.c_str()[13];//pega o numero da porta do EV3
}

//-----------------------------------------------------------------------------

Motor::Motor(enderecoPorta address, const tipoMotor &t)
{
	connect({{ "address", { address } }, { "driver_name", { t }}});
	port = address.c_str()[13];//pega o numero da porta do EV3

}

//-----------------------------------------------------------------------------

bool Motor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
		{
	static const std::string _strClassDir { SYS_ROOT "/tacho-motor/" };
	static const std::string _strPattern  { "motor" };

	try
	{
		return device::connect(_strClassDir, _strPattern, match);
	}
	catch (...) {  }

	path.clear();

	return false;
		}

//-----------------------------------------------------------------------------

MotorMedio::MotorMedio(enderecoPorta portaMotorLego) : Motor(portaMotorLego, motorMedio)
{
}

//-----------------------------------------------------------------------------

MotorGrande::MotorGrande(enderecoPorta portaMotorLego) : Motor(portaMotorLego, motorGrande)
{
}

//-----------------------------------------------------------------------------

Led::Led(std::string name)
{
	static const std::string _strClassDir { SYS_ROOT "/leds/" };
	connect(_strClassDir, name, std::map<std::string, std::set<std::string>>());
}

//-----------------------------------------------------------------------------

void Led::flash(unsigned tempoLigadoMilissegundos, unsigned tempoDesligadoMilissegundos)
{
	static const tipoModo timer("timer");
	setGatilho(timer);
	if (tempoLigadoMilissegundos)
	{
		// A workaround for ev3dev/ev3dev#225.
		// It takes some time for delay_{on,off} sysfs attributes to appear after
		// led trigger has been set to "timer".
		for (int i = 0; ; ++i) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			try {
				setAtrasoLigado(tempoLigadoMilissegundos );
				setAtrasoDesligado(tempoDesligadoMilissegundos);
				break;
			} catch(...) {
				if (i >= 5) throw;
			}
		}
	}
}


//~autogen leds-define platforms.ev3.led>currentClass

Led Led::vermelhoEsquerdo{"led0:red:brick-status"};
Led Led::vermelhoDireito{"led1:red:brick-status"};
Led Led::verdeEsquerdo{"led0:green:brick-status"};
Led Led::verdeDireito{"led1:green:brick-status"};

std::vector<Led*> Led::esquerdo{ &Led::vermelhoEsquerdo, &Led::verdeEsquerdo };
std::vector<Led*> Led::direito{ &Led::vermelhoDireito, &Led::verdeDireito };

std::vector<float> Led::apagado{ static_cast<float>(0), static_cast<float>(0) };
std::vector<float> Led::vermelho{ static_cast<float>(1), static_cast<float>(0) };
std::vector<float> Led::verde{ static_cast<float>(0), static_cast<float>(1) };
std::vector<float> Led::ambar{ static_cast<float>(1), static_cast<float>(1) };
std::vector<float> Led::laranja{ static_cast<float>(1), static_cast<float>(0.5) };
std::vector<float> Led::amarelo{ static_cast<float>(0.1), static_cast<float>(1) };

//-----------------------------------------------------------------------------
void Led::todosDesligados() {

	vermelhoEsquerdo.desligado();
	vermelhoDireito.desligado();
	verdeEsquerdo.desligado();
	verdeDireito.desligado();

}

//-----------------------------------------------------------------------------

void Led::setCor(const std::vector<Led*> &group, const std::vector<float> &color) {
	const size_t n = std::min(group.size(), color.size());
	for(size_t i = 0; i < n; ++i)
		group[i]->setBrilhoPorcentagem(color[i]);
}

//-----------------------------------------------------------------------------

power_supply power_supply::battery { "" };

//-----------------------------------------------------------------------------

power_supply::power_supply(std::string name)
{
	static const std::string _strClassDir { SYS_ROOT "/power_supply/" };

	if (name.empty())
		name = "legoev3-battery";

	connect(_strClassDir, name, std::map<std::string, std::set<std::string>>());
}

//-----------------------------------------------------------------------------

Botao::file_descriptor::file_descriptor(const char *path, int flags)
: _fd(open(path, flags))
{}

Botao::file_descriptor::~file_descriptor()
{
	if (_fd != -1) close(_fd);
}

//-----------------------------------------------------------------------------

Botao::Botao(int bit)
: _bit(bit),
  _buf((KEY_CNT + bits_per_long - 1) / bits_per_long),
  _fd( new file_descriptor("/dev/input/by-path/platform-gpio_keys-event", O_RDONLY) )
{ }

//-----------------------------------------------------------------------------

bool Botao::apertado() const
{
#ifndef NO_LINUX_HEADERS
	if (ioctl(*_fd, EVIOCGKEY(_buf.size()), _buf.data()) < 0)
	{
		// handle error
	}
#endif
	return (_buf[_bit / bits_per_long] & 1 << (_bit % bits_per_long));
}

//-----------------------------------------------------------------------------

bool Botao::process()
{
	bool new_state = apertado();

	if (new_state != _state) {
		_state = new_state;
		if (onclick) onclick(new_state);
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------

#ifndef NO_LINUX_HEADERS
Botao Botao::voltar (KEY_BACKSPACE);
Botao Botao::esquerdo (KEY_LEFT);
Botao Botao::direito(KEY_RIGHT);
Botao Botao::cima   (KEY_UP);
Botao Botao::baixo (KEY_DOWN);
Botao Botao::enter(KEY_ENTER);
#endif

//-----------------------------------------------------------------------------

bool Botao::process_all() {
	std::array<bool, 6> changed{{
		voltar. process(),
				esquerdo. process(),
				direito.process(),
				cima.   process(),
				baixo. process(),
				enter.process()
	}};
	return std::any_of(changed.begin(), changed.end(), [](bool c){ return c; });
}

//-----------------------------------------------------------------------------
//*************Classe Som *****************//
void Som::beep(const std::string &args, bool sincronizado)
{
	std::ostringstream cmd;
	cmd << "/usr/bin/beep " << args;
	if (!sincronizado) cmd << " &";
	int temp = std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

void Som::tone(
		const std::vector< std::vector<float> > &sequencia,
		bool sincronizado
)
{
	std::ostringstream args;
	bool first = true;

	for(auto v : sequencia) {
		if (first) {
			first = false;
		} else {
			args << " -n";
		}

		if (v.size() > 0) {
			args << " -f " << v[0];
		} else {
			continue;
		}

		if (v.size() > 1) {
			args << " -l " << v[1];
		} else {
			continue;
		}

		if (v.size() > 2) {
			args << " -D " << v[2];
		} else {
			continue;
		}
	}

	beep(args.str(), sincronizado);
}

//-----------------------------------------------------------------------------

void Som::tone(float frequencia, float ms, bool sincronizado) {
	tone({{frequencia, ms, 0.0f}}, sincronizado);
}

//-----------------------------------------------------------------------------

void Som::tocar(const std::string &arquivoAudio, bool sincronizado)
{
	std::ostringstream cmd;
	cmd << "/usr/bin/aplay -q " << arquivoAudio;

	if (!sincronizado) cmd << " &";

	int temp = std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

void Som::falar(const std::string &texto, bool sincronizado)
{
	std::ostringstream cmd;

	cmd << "/usr/bin/espeak -a 200 -s 140 -v pt-br --stdout \"" << texto << "\""
			<< " | /usr/bin/aplay -q";

	if (!sincronizado) cmd << " &";

	int temp = std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

lcd::lcd() :
		  _fb(nullptr),
		  _fbsize(0),
		  _llength(0),
		  _xres(0),
		  _yres(0),
		  _bpp(0)
{
	init();
}

//-----------------------------------------------------------------------------

lcd::~lcd()
{
	deinit();
}

//-----------------------------------------------------------------------------

void lcd::fill(unsigned char pixel)
{
	if (_fb && _fbsize)
	{
		memset(_fb, pixel, _fbsize);
	}
}

//-----------------------------------------------------------------------------

void lcd::init()
{
	using namespace std;

#ifdef _LINUX_FB_H
	int fbf = open("/dev/fb0", O_RDWR);
	if (fbf < 0)
		return;

	fb_fix_screeninfo i;
	if (ioctl(fbf, FBIOGET_FSCREENINFO, &i) < 0)
		return;

	_fbsize  = i.smem_len;
	_llength = i.line_length;

	_fb = (unsigned char*)mmap(NULL, _fbsize, PROT_READ|PROT_WRITE, MAP_SHARED, fbf, 0);
	if (_fb == nullptr)
		return;

	fb_var_screeninfo v;

	if (ioctl(fbf, FBIOGET_VSCREENINFO, &v) < 0)
		return;

	_xres = v.xres;
	_yres = v.yres;
	_bpp  = v.bits_per_pixel;
#endif
}

//-----------------------------------------------------------------------------

void lcd::deinit()
{
	if (_fb)
	{
		munmap(_fb, 0);
	}

	_fbsize = 0;
}

//-----------------------------------------------------------------------------

lego_port::lego_port(enderecoPorta address)
{
	connect({{ "address", { address } }});
}

//-----------------------------------------------------------------------------

bool lego_port::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
		{
	static const std::string _strClassDir { SYS_ROOT "/lego-port/" };
	static const std::string _strPattern  { "port" };

	try
	{
		return device::connect(_strClassDir, _strPattern, match);
	}
	catch (...) { }

	path.clear();

	return false;
		}

//-----------------------------------------------------------------------------

} // namespace ev3dev

// vim: sw=2
