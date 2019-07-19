/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 *
 * Copyright (c) 2014 - Franz Detro
 * 2018 - Modificada, adaptada e traduzida por - Julio Cesar Goldner Vendramini
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
 * 2018-11-16 - Primeira modificação
 * 				Simplificação e eliminação de sensores que não serão utilizados nunca no Ifes Colatina
 * 				Modificação para o robô falar qual porta está com problemas
 * 				Aviso de qual o problema no lançamento de erro
 * 2018-11-17 - Identação corrigida
 * 				Sensor de Toque traduzido
 * 				Classe Som traduzida
 * 				Criação da Classe Cor(substitui a tupla), tradução da Classe Sensor Cor
 * 				Tradução dos métodos e atributos da classe Motor, faltam os comentários
 * 				Tradução da classe Botão, Led, SensorGiroscópio, SensorInfravermelho
 *
 */


#ifndef EV3DEV_H_
#define EV3DEV_H_

#pragma once

#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>
#include <algorithm>
#include <functional>
#include <memory>

//-----------------------------------------------------------------------------

namespace ev3dev {


//Classe Genérica e dispositivos, colocado em outro arquivo para facilitar
//a navegação
#include "ev3dev-device.h"



/// Sensor de Toque
class SensorToque : public sensor
{
public:
	SensorToque(enderecoPorta portaSensor);

	// Button state
	static constexpr char modo_toque[] = "TOUCH";


	// Um booleano indicando se o botão do sensor está apertado ou não
	//retorna true se estiver apertado
	bool apertado(bool mudarModo = true) {
		if (mudarModo) setModo(modo_toque);
		return value(0);
	}

};


class Cor{
	public:
	Cor(int vermelha, int verde, int azul){
		this->vermelha = vermelha;
		this->verde = verde;
		this->azul = azul;
	}
	int vermelha;
	int verde;
	int azul;
};

/**
// LEGO EV3 Sensor de Cor.
 * Importante detalhas que aqui usamos as cores com os nomes em Inglẽs
 * Red - Vermelho
 * Green - Verde
 * Blue - Azul
 *
 */
class SensorCor : public sensor
{
public:
	SensorCor(enderecoPorta portaSensor);

	// luz de reflexão. Led Vermelho Ligado.
	static constexpr char modo_col_reflect[] = "COL-REFLECT";

	// Luz ambiente Led Vermelho Desligado.
	static constexpr char modo_col_ambient[] = "COL-AMBIENT";

	// Modo detecção de cor. Todos os Leds piscam rapidamente, a cor parece ser branca.
	static constexpr char modo_col_color[] = "COL-COLOR";

	// Reflexão bruta. Led Vermelho Ligado.
	static constexpr char modo_ref_raw[] = "REF-RAW";

	// Componentes de Cor Bruto (Red-Green-Blue). Todos os Leds piscam rapidamente, a cor parece ser branca.
	static constexpr char modo_rgb_raw[] = "RGB-RAW";

	// Sem cor.
	static constexpr char corSemCor[] = "NoColor";

	// Cor Preta
	static constexpr char corPreta[] = "Black";

	// Cor Azul.
	static constexpr char corAzul[] = "Blue";

	// Cor Verde.
	static constexpr char corVerde[] = "Green";

	// Cor Amarela.
	static constexpr char corAmarela[] = "Yellow";

	// Cor Vermelha.
	static constexpr char corVermelha[] = "Red";

	// Cor Branca.
	static constexpr char corBranca[] = "White";

	// Cor Marrom.
	static constexpr char corMarrom[] = "Brown";


	// Reflexão de luz em porcentagem. Luz vermelha ligada.
	int intensidadeLuzRefletida(bool mudarModo = true) {
		if (mudarModo) setModo(modo_col_reflect);
		return value(0);
	}

	// Intensidade de Luz ambiente. A luz do sensor é um azul fraco.
	int ambient_light_intensity(bool mudarModo = true) {
		if (mudarModo) setModo(modo_col_ambient);
		return value(0);
	}

	/** Cores detectadas pelo sensor, classificadas por valores.
	 *   - 0: No color
	 *   - 1: Preta
	 *   - 2: Azul
	 *   - 3: Verde
	 *   - 4: Amarela
	 *   - 5: Vermelha
	 *   - 6: Branca
	 *   - 7: Marrom
	 *
	 */
	int cor(bool mudarModo = true) {
		if (mudarModo) setModo(modo_col_color);
		return value(0);
	}

	//Retorna os componentes da cor detectada
	//Vermelha, verde e azul, valores entre 0-1020.
	Cor corBruta(bool mudarModo = true) {
		if (mudarModo) setModo(modo_rgb_raw);

		return Cor( value(0), value(1), value(2) );
	}

	// Retorna componente da cor vermelha, valor entre 0-1020.
	int vermelha(bool mudarModo = true) {
		if (mudarModo) setModo(modo_rgb_raw);
		return value(0);
	}

	// Retorna componente da cor verde, valor entre 0-1020.
	int verde(bool mudarModo = true) {
		if (mudarModo) setModo(modo_rgb_raw);
		return value(1);
	}

	// Retorna componente da cor azul, valor entre 0-1020.
	int azul(bool mudarModo = true) {
		if (mudarModo) setModo(modo_rgb_raw);
		return value(2);
	}

};


//~autogen special-sensor-declaration specialSensorTypes.ultrasonicSensor>currentClass

// LEGO EV3 ultrasonic sensor.
class SensorUltrassonico : public sensor
{
public:
	SensorUltrassonico(enderecoPorta address = INPUT_AUTO);

	SensorUltrassonico(enderecoPorta address, const std::set<tipoSensor>& sensorTypes);

	// Medição contínua em centímetros
	static constexpr char modo_cont_dist_cm[] = "US-DIST-CM";

	// Listen.
	static constexpr char modo_ouvindo[] = "US-LISTEN";

	// Medição única em centímetros
	static constexpr char modo_unico_dist_cm[] = "US-SI-CM";



	// Measurement of the distance detected by the sensor,
	// in centimeters.
	float distancia(bool mudarModo = true) {
		if (mudarModo) setModo(modo_cont_dist_cm);
		return float_value(0);
	}
	// Value indicating whether another ultrasonic sensor could
	// be heard nearby.
	bool outroSensorPresente(bool mudarModo = true) {
		if (mudarModo) setModo(modo_ouvindo);
		return value(0);
	}

};


//Sensor de gisrocópio para LEGO EV3
class SensorGiroscopio : public sensor
{
public:
	SensorGiroscopio(enderecoPorta portaSensor);

	// Angulo desde o inicio do modo
	static constexpr char modoAngulo[] = "GYRO-ANG";

	// Velocidade de rotação
	static constexpr char modoRotacaoPorSegundo[] = "GYRO-RATE";

	// Raw sensor value (melhor não usar)
	//static constexpr char mode_gyro_fas[] = "GYRO-FAS";

	// Angulo e velocidade de rotacao
	static constexpr char modoAnguloERotacaoPorSegundo[] = "GYRO-G&A";

	// Calibration ??? (ainda são sei o que é)
	//static constexpr char mode_gyro_cal[] = "GYRO-CAL";


	// Números de graus que o sensor girou desde que entrou nesse modo
	int angulo(bool mudarModo = true) {
		if (mudarModo) setModo(modoAngulo);
		return value(0);
	}


	// A velocidade de rotação em graus/segundo do sensor
	int grausPorSegundo(bool mudarModo = true) {
		if (mudarModo) setModo(modoRotacaoPorSegundo);
		return value(0);
	}

	// Angulo (graus) and Velocidade de rotacao(angulo/segundo).
	std::tuple<int, int> rate_and_angle(bool mudarModo = true) {
		if (mudarModo) setModo(modoAnguloERotacaoPorSegundo);
		return std::make_tuple( value(0), value(1) );
	}

};

//Sensor de infravermelho para LEGO EV3.
//Opções de controle remoto desativadas
class SensorInfravermelho : public sensor
{
public:
	SensorInfravermelho(enderecoPorta portaSensor);

	// Proximidade
	static constexpr char modo_proximidade[] = "IR-PROX";

	// IR Seeker (ainda sem saber o que é)
	static constexpr char modo_ir_seek[] = "IR-SEEK";

	// Calibration ??? (ainda sem saber o que é)
	static constexpr char modo_ir_cal[] = "IR-CAL";


	// A measurement of the distance between the sensor and the remote,
	// as a percentage. 100% is approximately 70cm/27in.
	int proximity(bool mudarModo = true) {
		if (mudarModo) setModo(modo_proximidade);
		return (int)(value(0)*0.7);
	}

};
//-----------------------------------------------------------------------------

// The motor class provides a uniform interface for using motors with
// positional and directional feedback such as the EV3 and NXT motors.
// This feedback allows for precise control of the motors. This is the
// most common type of motor, so we just call it `motor`.
//
// The way to configure a motor is to set the '_sp' attributes when
// calling a command or before. Only in 'run_direct' mode attribute
// changes are processed immediately, in the other modes they only
// take place when a new command is issued.

//As opções de controle de PID estão removidas.
//As opções de dutyCycle estão removidas.
class Motor : protected device
{
public:
	typedef tipoDispositivo tipoMotor;

	Motor(enderecoPorta);
	Motor(enderecoPorta, const tipoMotor&);

	static constexpr char motorGrande[]  = "lego-ev3-l-motor";
	static constexpr char motorMedio[] = "lego-ev3-m-motor";

	using device::connected;
	using device::device_index;

	//~autogen generic-declare-property-value classes.motor>currentClass

	// Run the motor until another command is sent.
	static constexpr char comandoGiraParaSempre[] = "run-forever";

	// Run to an absolute position specified by `position_sp` and then
	// stop using the action specified in `stop_action`.
	static constexpr char comandoGiraParaPosicaoAbsoluta[] = "run-to-abs-pos";

	// Run to a position relative to the current `position` value.
	// The new position will be current `position` + `position_sp`.
	// When the new position is reached, the motor will stop using
	// the action specified by `stop_action`.
	static constexpr char comandoGiraParaPosicaoRelativa[] = "run-to-rel-pos";

	// Run the motor for the amount of time specified in `time_sp`
	// and then stop the motor using the action specified by `stop_action`.
	static constexpr char comandoGiraPorTempoDeterminado[] = "run-timed";

	// Run the motor at the duty cycle specified by `duty_cycle_sp`.
	// Unlike other run commands, changing `duty_cycle_sp` while running *will*
	// take effect immediately.
	static constexpr char comandoGiraDireto[] = "run-direct";

	// Stop any of the run commands before they are complete using the
	// action specified by `stop_action`.
	static constexpr char comandoParar[] = "stop";

	// Reset all of the motor parameter attributes to their default value.
	// This will also have the effect of stopping the motor.
	static constexpr char comandoReset[] = "reset";

	// Sets the normal polarity of the rotary encoder.
	static constexpr char polaridadeEncoderNormal[] = "normal";

	// Sets the inversed polarity of the rotary encoder.
	static constexpr char polaridadeEncoderInvertida[] = "inversed";

	// With `normal` polarity, a positive duty cycle will
	// cause the motor to rotate clockwise.
	static constexpr char polaridadeNormal[] = "normal";

	// With `inversed` polarity, a positive duty cycle will
	// cause the motor to rotate counter-clockwise.
	static constexpr char polaridadeInvertida[] = "inversed";

	// Power is being sent to the motor.
	static constexpr char estadoGirando[] = "running";

	// The motor is ramping up or down and has not yet reached a constant output level.
	static constexpr char estadoAcelerando[] = "ramping";

	// The motor is not turning, but rather attempting to hold a fixed position.
	static constexpr char estadoMantendoPosicao[] = "holding";

	// The motor is turning, but cannot reach its `speed_sp`.
	static constexpr char estadoSobrecarregado[] = "overloaded";

	// The motor is not turning when it should be.
	static constexpr char estadoTravado[] = "stalled";

	// Power will be removed from the motor and it will freely coast to a stop.
	static constexpr char acaoParadaLivre[] = "coast";

	// Power will be removed from the motor and a passive electrical load will
	// be placed on the motor. This is usually done by shorting the motor terminals
	// together. This load will absorb the energy from the rotation of the motors and
	// cause the motor to stop more quickly than coasting.
	static constexpr char acaoParadaFreio[] = "brake";

	// Does not remove power from the motor. Instead it actively try to hold the motor
	// at the current position. If an external force tries to turn the motor, the motor
	// will `push back` to maintain its position.
	static constexpr char acaoParadaTravar[] = "hold";


	//~autogen

	//~autogen generic-get-set classes.motor>currentClass

	// Address: read-only
	// Returns the name of the port that this motor is connected to.
	std::string getPortaConectada() const { return get_attr_string("address"); }

	// Command: write-only
	// Sends a command to the motor controller. See `commands` for a list of
	// possible values.
	auto setComando(std::string comando) -> decltype(*this) {
		set_attr_string("command", comando);
		return *this;
	}

	// Commands: read-only
	// Returns a list of commands that are supported by the motor
	// controller. Possible values are `run-forever`, `run-to-abs-pos`, `run-to-rel-pos`,
	// `run-timed`, `run-direct`, `stop` and `reset`. Not all commands may be supported.
	//
	// - `run-forever` will cause the motor to run until another command is sent.
	// - `run-to-abs-pos` will run to an absolute position specified by `position_sp`
	//   and then stop using the action specified in `stop_action`.
	// - `run-to-rel-pos` will run to a position relative to the current `position` value.
	//   The new position will be current `position` + `position_sp`. When the new
	//   position is reached, the motor will stop using the action specified by `stop_action`.
	// - `run-timed` will run the motor for the amount of time specified in `time_sp`
	//   and then stop the motor using the action specified by `stop_action`.
	// - `run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.
	//   Unlike other run commands, changing `duty_cycle_sp` while running *will*
	//   take effect immediately.
	// - `stop` will stop any of the run commands before they are complete using the
	//   action specified by `stop_action`.
	// - `reset` will reset all of the motor parameter attributes to their default value.
	//   This will also have the effect of stopping the motor.
	mode_set getComandos() const { return get_attr_set("commands"); }

	// Count Per Rot: read-only
	// Returns the number of tacho counts in one rotation of the motor. Tacho counts
	// are used by the position and speed attributes, so you can use this value
	// to convert rotations or degrees to tacho counts. (rotation motors only)
	int pulsosPorVolta() const { return get_attr_int("count_per_rot"); }

	// Driver Name: read-only
	// Returns the name of the driver that provides this tacho motor device.
	std::string nomeDriver() const { return get_attr_string("driver_name"); }


	// Polarity: read/write
	// Sets the polarity of the motor. With `normal` polarity, a positive duty
	// cycle will cause the motor to rotate clockwise. With `inversed` polarity,
	// a positive duty cycle will cause the motor to rotate counter-clockwise.
	// Valid values are `normal` and `inversed`.
	std::string getPolaridade() const { return get_attr_string("polarity"); }
	auto setPolaridade(std::string polaridade) -> decltype(*this) {
		set_attr_string("polarity", polaridade);
		return *this;
	}

	// Position: read/write
	// Returns the current position of the motor in pulses of the rotary
	// encoder. When the motor rotates clockwise, the position will increase.
	// Likewise, rotating counter-clockwise causes the position to decrease.
	// Writing will set the position to that value.
	int getPosicaoAtual() const { return get_attr_int("position"); }
	auto setPosicaoAtual(int graus) -> decltype(*this) {
		set_attr_int("position", graus);
		return *this;
	}

	// Position SP: read/write
	// Writing specifies the target position for the `run-to-abs-pos` and `run-to-rel-pos`
	// commands. Reading returns the current value. Units are in tacho counts. You
	// can use the value returned by `counts_per_rot` to convert tacho counts to/from
	// rotations or degrees.
	int getPosicaoDesejada() const { return get_attr_int("position_sp"); }
	auto setPosicaoDesejada(int graus) -> decltype(*this) {
		set_attr_int("position_sp", graus);
		return *this;
	}

	// Max Speed: read-only
	// Returns the maximum value that is accepted by the `speed_sp` attribute. This
	// may be slightly different than the maximum speed that a particular motor can
	// reach - it's the maximum theoretical speed.
	int getVelocidadeMaxima() const { return get_attr_int("max_speed"); }

	// Speed: read-only
	// Returns the current motor speed in tacho counts per second. Note, this is
	// not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`
	// attribute to convert this value to RPM or deg/sec.
	int getVelocidadeAtual() const { return get_attr_int("speed"); }

	// Speed SP: read/write
	// Writing sets the target speed in tacho counts per second used for all `run-*`
	// commands except `run-direct`. Reading returns the current value. A negative
	// value causes the motor to rotate in reverse with the exception of `run-to-*-pos`
	// commands where the sign is ignored. Use the `count_per_rot` attribute to convert
	// RPM or deg/sec to tacho counts per second. Use the `count_per_m` attribute to
	// convert m/s to tacho counts per second.
	int getVelocidadeDesejada() const { return get_attr_int("speed_sp"); }
	auto setVelocidadeDesejada(int grausPorSegundo) -> decltype(*this) {
		set_attr_int("speed_sp", grausPorSegundo);
		return *this;
	}

	// Ramp Up SP: read/write
	// Writing sets the ramp up setpoint. Reading returns the current value. Units
	// are in milliseconds and must be positive. When set to a non-zero value, the
	// motor speed will increase from 0 to 100% of `max_speed` over the span of this
	// setpoint. The actual ramp time is the ratio of the difference between the
	// `speed_sp` and the current `speed` and max_speed multiplied by `ramp_up_sp`.
	int getRampaAceleracaoDesejada() const { return get_attr_int("ramp_up_sp"); }
	auto setRampaAceleracaoDesejada(int milissegundos) -> decltype(*this) {
		set_attr_int("ramp_up_sp", milissegundos);
		return *this;
	}

	// Ramp Down SP: read/write
	// Writing sets the ramp down setpoint. Reading returns the current value. Units
	// are in milliseconds and must be positive. When set to a non-zero value, the
	// motor speed will decrease from 0 to 100% of `max_speed` over the span of this
	// setpoint. The actual ramp time is the ratio of the difference between the
	// `speed_sp` and the current `speed` and max_speed multiplied by `ramp_down_sp`.
	int getRampaDesaceleracaoDesejada() const { return get_attr_int("ramp_down_sp"); }
	auto setRampaDesaceleracaoDesejada(int milissegundos) -> decltype(*this) {
		set_attr_int("ramp_down_sp", milissegundos);
		return *this;
	}

    // State: read-only
	// Reading returns a list of state flags. Possible flags are
	// `running`, `ramping`, `holding`, `overloaded` and `stalled`.
	mode_set getEstados() const { return get_attr_set("state"); }


	//Verifica se o motor está travado, ou seja
	//Esta tentando se movimentar mas não possui força suficiente
	bool estaTravado() const {
		mode_set retorno = this->getEstados();
		if(retorno.find(Motor::estadoTravado)!=retorno.end()) return true;
		else return false;
	}
	//Verifica se o motor está girando
	bool estaGirando() const {
		mode_set retorno = this->getEstados();
		if(retorno.find(Motor::estadoGirando)!=retorno.end()) return true;
		else return false;
	}

	//Verifica se o motor está sobrecarregado
	//Ou seja, não está conseguindo atingir a velocidade solicitada
	bool estaSobrecarregado() const {
		mode_set retorno = this->getEstados();
		if(retorno.find(Motor::estadoSobrecarregado)!=retorno.end()) return true;
		else return false;
	}

	// Stop Action: read/write
	// Reading returns the current stop action. Writing sets the stop action.
	// The value determines the motors behavior when `command` is set to `stop`.
	// Also, it determines the motors behavior when a run command completes. See
	// `stop_actions` for a list of possible values.
	std::string getAcaoParada() const { return get_attr_string("stop_action"); }
	auto setAcaoParada(std::string acao) -> decltype(*this) {
		set_attr_string("stop_action", acao);
		return *this;
	}

	// Stop Actions: read-only
	// Returns a list of stop actions supported by the motor controller.
	// Possible values are `coast`, `brake` and `hold`. `coast` means that power will
	// be removed from the motor and it will freely coast to a stop. `brake` means
	// that power will be removed from the motor and a passive electrical load will
	// be placed on the motor. This is usually done by shorting the motor terminals
	// together. This load will absorb the energy from the rotation of the motors and
	// cause the motor to stop more quickly than coasting. `hold` does not remove
	// power from the motor. Instead it actively tries to hold the motor at the current
	// position. If an external force tries to turn the motor, the motor will 'push
	// back' to maintain its position.
	mode_set getListaAcoesParada() const { return get_attr_set("stop_actions"); }

	// Time SP: read/write
	// Writing specifies the amount of time the motor will run when using the
	// `run-timed` command. Reading returns the current value. Units are in
	// milliseconds.
	int getTempoLigadoDesejado() const { return get_attr_int("time_sp"); }
	auto setTempoLigadoDesejado(int v) -> decltype(*this) {
		set_attr_int("time_sp", v);
		return *this;
	}

	// Run the motor until another command is sent.
	void giraParaSempre() { setComando("run-forever"); }

	// Run to an absolute position specified by `position_sp` and then
	// stop using the action specified in `stop_action`.
	void giraParaPosicaoAbsoluta() { setComando("run-to-abs-pos"); }

	// Run to a position relative to the current `position` value.
	// The new position will be current `position` + `position_sp`.
	// When the new position is reached, the motor will stop using
	// the action specified by `stop_action`.
	void giraParaPosicaoRelativa() { setComando("run-to-rel-pos"); }

	// Run the motor for the amount of time specified in `time_sp`
	// and then stop the motor using the action specified by `stop_action`.
	void giraPorTempoDeterminado() { setComando("run-timed"); }

	// Run the motor at the duty cycle specified by `duty_cycle_sp`.
	// Unlike other run commands, changing `duty_cycle_sp` while running *will*
	// take effect immediately.
	void run_direct() { setComando("run-direct"); }

	// Stop any of the run commands before they are complete using the
	// action specified by `stop_action`.
	void parar() { setComando("stop"); }

	// Reset all of the motor parameter attributes to their default value.
	// This will also have the effect of stopping the motor.
	void reset() { setComando("reset"); }


	//~autogen

protected:
	Motor() {}

	bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
};

//-----------------------------------------------------------------------------

// Motor médio do EV3
class MotorMedio : public Motor
{
public:
	MotorMedio(enderecoPorta portaMotor);
};

//-----------------------------------------------------------------------------

// Motor grande do EV3
class MotorGrande : public Motor
{
public:
	MotorGrande(enderecoPorta portaMotor);
};

//-----------------------------------------------------------------------------

// Any device controlled by the generic LED driver.
// See https://www.kernel.org/doc/Documentation/leds/leds-class.txt
// for more details.
class Led : protected device
{
public:
	Led(std::string nome);

	using device::connected;

	//~autogen generic-get-set classes.led>currentClass

	// Max Brightness: read-only
	// Returns the maximum allowable brightness value.
	int getBrilhoMaximo() const { return get_attr_int("max_brightness"); }

	// Brightness: read/write
	// Sets the brightness level. Possible values are from 0 to `max_brightness`.
	int getBrilho() const { return get_attr_int("brightness"); }
	auto setBrilho(int intensidade) -> decltype(*this) {
		set_attr_int("brightness", intensidade);
		return *this;
	}

	// Triggers: read-only
	// Returns a list of available triggers.
	mode_set getGatilhos() const { return get_attr_set("trigger"); }

	// Trigger: read/write
	// Sets the led trigger. A trigger
	// is a kernel based source of led events. Triggers can either be simple or
	// complex. A simple trigger isn't configurable and is designed to slot into
	// existing subsystems with minimal additional code. Examples are the `ide-disk` and
	// `nand-disk` triggers.
	//
	// Complex triggers whilst available to all LEDs have LED specific
	// parameters and work on a per LED basis. The `timer` trigger is an example.
	// The `timer` trigger will periodically change the LED brightness between
	// 0 and the current brightness setting. The `on` and `off` time can
	// be specified via `delay_{on,off}` attributes in milliseconds.
	// You can change the brightness value of a LED independently of the timer
	// trigger. However, if you set the brightness value to 0 it will
	// also disable the `timer` trigger.
	std::string getGatilho() const { return get_attr_from_set("trigger"); }
	auto setGatilho(std::string gatilho) -> decltype(*this) {
		set_attr_string("trigger", gatilho);
		return *this;
	}

	// Delay On: read/write
	// The `timer` trigger will periodically change the LED brightness between
	// 0 and the current brightness setting. The `on` time can
	// be specified via `delay_on` attribute in milliseconds.
	int getAtrasoLigado() const { return get_attr_int("delay_on"); }
	auto setAtrasoLigado(int milissegundos) -> decltype(*this) {
		set_attr_int("delay_on", milissegundos);
		return *this;
	}

	// Delay Off: read/write
	// The `timer` trigger will periodically change the LED brightness between
	// 0 and the current brightness setting. The `off` time can
	// be specified via `delay_off` attribute in milliseconds.
	int getAtrasoDesligado() const { return get_attr_int("delay_off"); }
	auto setAtrasoDesligado(int milissegundos) -> decltype(*this) {
		set_attr_int("delay_off", milissegundos);
		return *this;
	}


	//~autogen

	// Gets the LED's brightness as a percentage (0-1) of the maximum.
	float getBrilhoPorcentagem() const {
		return static_cast<float>(getBrilho()) / getBrilhoMaximo();
	}

	// Sets the LED's brightness as a percentage (0-1) of the maximum.
	auto setBrilhoPorcentagem(float porcentagem) -> decltype(*this) {
		return setBrilho(porcentagem * getBrilhoMaximo());
	}

	// Turns the led on by setting its brightness to the maximum level.
	void ligado()  { setBrilho(getBrilhoMaximo()); }

	// Turns the led off.
	void desligado() { setBrilho(0); }

	// Enables timer trigger and sets delay_on and delay_off attributes to the
	// provided values (in milliseconds).
	void flash(unsigned tempoLigadoMilissegundos, unsigned tempoDesligadoMilissegundos);
	//~autogen leds-declare platforms.ev3.led>currentClass

	static Led vermelhoEsquerdo;
	static Led vermelhoDireito;
	static Led verdeEsquerdo;
	static Led verdeDireito;

	static std::vector<Led*> esquerdo;
	static std::vector<Led*> direito;

	static std::vector<float> apagado;
	static std::vector<float> vermelho;
	static std::vector<float> verde;
	static std::vector<float> ambar;
	static std::vector<float> laranja;
	static std::vector<float> amarelo;


	// Assigns to each led in `group` corresponding brightness percentage from `color`.
	static void setCor(const std::vector<Led*> &group, const std::vector<float> &color);

	static void todosDesligados();

protected:
	int brilhoMaximo = 0;
};

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.powerSupply>currentClass

// A generic interface to read data from the system's power_supply class.
// Uses the built-in legoev3-battery if none is specified.

//~autogen
class power_supply : protected device
{
public:
	power_supply(std::string name);

	using device::connected;

	//~autogen generic-get-set classes.powerSupply>currentClass

	// Measured Current: read-only
	// The measured current that the battery is supplying (in microamps)
	int measured_current() const { return get_attr_int("current_now"); }

	// Measured Voltage: read-only
	// The measured voltage that the battery is supplying (in microvolts)
	int measured_voltage() const { return get_attr_int("voltage_now"); }

	// Max Voltage: read-only
	int max_voltage() const { return get_attr_int("voltage_max_design"); }

	// Min Voltage: read-only
	int min_voltage() const { return get_attr_int("voltage_min_design"); }

	// Technology: read-only
	std::string technology() const { return get_attr_string("technology"); }

	// Type: read-only
	std::string type() const { return get_attr_string("type"); }


	//~autogen

	float measured_amps()       const { return measured_current() / 1000000.f; }
	float measured_volts()      const { return measured_voltage() / 1000000.f; }

	static power_supply battery;
};

//-----------------------------------------------------------------------------

// Botões do EV3
class Botao
{
public:
	Botao(int bit);

	// Verifica se o Botao está apertado.
	bool apertado() const;



	static Botao voltar;
	static Botao esquerdo;
	static Botao direito;
	static Botao cima;
	static Botao baixo;
	static Botao enter;



private:
	int _bit;
	bool _state = false;
	std::vector<unsigned long> _buf;

	/*Sem acesso ao desenvolvedor*/
	// Gets called whenever the Botao state changes.
	// The user has to call the process() function to check for state change.
	std::function<void(bool)> onclick;

	// Check if the Botao state has changed,
	// call onclick function in case it has.
	// Returns true if the state has changed since the last call.
	bool process();


	// Call process() for each of the EV3 Botaos.
	// Returns true if any of the states have changed since the last call.
	static bool process_all();


	struct file_descriptor {
		int _fd;

		file_descriptor(const char *path, int flags);
		~file_descriptor();
		operator int() { return _fd; }
	};

	std::shared_ptr<file_descriptor> _fd;
};

//-----------------------------------------------------------------------------

// EV3 Sound
class Som
{
public:
	static void beep(const std::string &args = "", bool sincronizado = false);
	static void tone(float frequencia, float ms, bool sincronizado = false);
	static void tone(const std::vector< std::vector<float> > &sequencia, bool sincronizado = false);
	static void tocar(const std::string &arquivoAudio, bool sincronizado = false);
	static void falar(const std::string &texto, bool sincronizado = false);
};

//-----------------------------------------------------------------------------

// EV3 LCD
class lcd
{
public:
	lcd();
	~lcd();

	bool available() const { return _fb != nullptr; }

	uint32_t resolution_x()   const { return _xres; }
	uint32_t resolution_y()   const { return _yres; }
	uint32_t bits_per_pixel() const { return _bpp; }

	uint32_t frame_buffer_size() const { return _fbsize; }
	uint32_t line_length()       const { return _llength; }

	unsigned char *frame_buffer() { return _fb; }

	void fill(unsigned char pixel);

protected:
	void init();
	void deinit();

private:
	unsigned char *_fb;
	uint32_t _fbsize;
	uint32_t _llength;
	uint32_t _xres;
	uint32_t _yres;
	uint32_t _bpp;
};

//-----------------------------------------------------------------------------

} // namespace ev3dev

// vim: sw=2


#endif /* EV3DEV_H_ */
