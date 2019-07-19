/*
 * movimento.h
 *
 *  Created on: 17/11/2018
 *      Author: ev3
 */

#ifndef MOVIMENTO_H_
#define MOVIMENTO_H_
#include "ev3dev.h"
using namespace ev3dev;
#define PI 3.14159265359
class Movimento{
private:
	Motor *motorEsquerdo, *motorDireito;
	double distanciaEntreEixos;
	double diametroRodas = 5.6;
	double larguraRodas = 2.8;
	double distanciaPercorridaPorGiroRoda;

	//double quantidadePulsosPorGrauRotacao;
	double granhoEngrenagem;
public:

	static constexpr int modoGraus = 1;
	static constexpr int modoCentimetros = 2;


	Movimento(enderecoPorta ladoEsquerdo, enderecoPorta ladoDireito){
		motorEsquerdo = new Motor(ladoEsquerdo, Motor::motorMedio);
		motorDireito = new Motor(ladoDireito, Motor::motorMedio);
		motorEsquerdo->reset();
		motorDireito->reset();
	}
	void configuraMotores(tipoModo polaridadeMotorEsquerdo,tipoModo polaridadeMotorDireito, tipoModo acaoParada,
			int rampaAceleracao=0, int rampaDesaceleracao=0)
	{
		motorEsquerdo->setPolaridade(polaridadeMotorEsquerdo);
		motorDireito->setPolaridade(polaridadeMotorDireito);
		motorEsquerdo->setAcaoParada(acaoParada);
		motorDireito->setAcaoParada(acaoParada);
		motorEsquerdo->setRampaAceleracaoDesejada(rampaAceleracao);
		motorDireito->setRampaAceleracaoDesejada(rampaAceleracao);
		motorEsquerdo->setRampaDesaceleracaoDesejada(rampaDesaceleracao);
		motorDireito->setRampaDesaceleracaoDesejada(rampaDesaceleracao);
	}

	void configuraEstrutura(double diametroRodas, double larguraRodas, double distanciaExternaEntreEixos, double ganhoEngrenagem=1){
		this->diametroRodas = diametroRodas;
		this->larguraRodas = larguraRodas;
		this->distanciaEntreEixos = distanciaExternaEntreEixos;
		this->granhoEngrenagem = ganhoEngrenagem;
		distanciaPercorridaPorGiroRoda = PI*diametroRodas*ganhoEngrenagem;
		//quantidadePulsosPorGrauRotacao=(distanciaExternaEntreEixos-larguraRodas)*distanciaPercorridaPorGiroRoda*(360/ganhoEngrenagem);

	}

	double getDistanciaPorGrau(){
		return distanciaPercorridaPorGiroRoda/360.0;
	}

	void setVelocidade(int velocidade, int modo = Movimento::modoGraus){
		if(modo==Movimento::modoGraus){
			motorDireito->setVelocidadeDesejada(velocidade);
			motorEsquerdo->setVelocidadeDesejada(velocidade);
		}else if(modo==Movimento::modoCentimetros){
			//envia o valor correto para centimetros por segundo
			motorDireito->setVelocidadeDesejada(velocidade/getDistanciaPorGrau());
			motorEsquerdo->setVelocidadeDesejada(velocidade/getDistanciaPorGrau());
		}
	}

	void andar(int distancia, int modo = Movimento::modoGraus){
		if(modo==Movimento::modoGraus){
			motorEsquerdo->setPosicaoDesejada(distancia);
			motorDireito->setPosicaoDesejada(distancia);
			motorEsquerdo->giraParaPosicaoRelativa();
			motorDireito->giraParaPosicaoRelativa();
			esperarAndar();
		}else if(modo==Movimento::modoCentimetros){
			motorEsquerdo->setPosicaoDesejada(distancia/getDistanciaPorGrau());
			motorDireito->setPosicaoDesejada(distancia/getDistanciaPorGrau());
			motorEsquerdo->giraParaPosicaoRelativa();
			motorDireito->giraParaPosicaoRelativa();
			esperarAndar();
		}
	}


	void andarAteValorAbsoluto(int distanciaEsquerdo, int distanciaDireito, int modo = Movimento::modoGraus){
			if(modo==Movimento::modoGraus){
				motorEsquerdo->setPosicaoDesejada(distanciaEsquerdo);
				motorDireito->setPosicaoDesejada(distanciaDireito);
				motorEsquerdo->giraParaPosicaoAbsoluta();
				motorDireito->giraParaPosicaoAbsoluta();
				esperarAndar();
			}else if(modo==Movimento::modoCentimetros){
				motorEsquerdo->setPosicaoDesejada(distanciaEsquerdo/getDistanciaPorGrau());
				motorDireito->setPosicaoDesejada(distanciaDireito/getDistanciaPorGrau());
				motorEsquerdo->giraParaPosicaoAbsoluta();
				motorDireito->giraParaPosicaoAbsoluta();
				esperarAndar();
			}
		}


	void andarMotorEsquerdo(int distancia, int modo = Movimento::modoGraus){
		if(modo==Movimento::modoGraus){
			motorEsquerdo->setPosicaoDesejada(distancia);
			motorEsquerdo->giraParaPosicaoRelativa();
			esperarAndar();
		}else if(modo==Movimento::modoCentimetros){
			motorEsquerdo->setPosicaoDesejada(distancia/getDistanciaPorGrau());
			motorEsquerdo->giraParaPosicaoRelativa();
			esperarAndar();
		}
	}

	void andarMotorDireito(int distancia, int modo = Movimento::modoGraus){
		if(modo==Movimento::modoGraus){
			motorDireito->setPosicaoDesejada(distancia);
			motorDireito->giraParaPosicaoRelativa();
			esperarAndar();
		}else if(modo==Movimento::modoCentimetros){
			motorDireito->setPosicaoDesejada(distancia/getDistanciaPorGrau());
			motorDireito->giraParaPosicaoRelativa();
			esperarAndar();
		}
	}

	/**
	//Esta função faz o robô andar em formato de círculo
	 * raioCirculo não pode ser menor que o entreEixo do robô
	 * grausSemiCirculo é o valor do semi circulo, ou seja, quantos graus quer girar
	 */
	void circuloParaEsquerda(double raioCirculo, double grausSemiCirculo){
		//a roda direita deve girar mais rapida
		if(raioCirculo < (distanciaEntreEixos-larguraRodas)){
			raioCirculo = (distanciaEntreEixos-larguraRodas);
		}
		double velE = motorEsquerdo->getVelocidadeDesejada();
		double distanciaMotorEsquerdo = (raioCirculo-(distanciaEntreEixos-larguraRodas))*2*PI*(grausSemiCirculo/360.0);
		double distanciaMotorDireito = (raioCirculo*2*PI)*(grausSemiCirculo/360.0);
		double velocidadeEsquerdo = velE*(distanciaMotorEsquerdo/distanciaMotorDireito);
		motorEsquerdo->setVelocidadeDesejada(velocidadeEsquerdo);
		motorEsquerdo->setPosicaoDesejada(distanciaMotorEsquerdo/getDistanciaPorGrau());
		motorDireito->setPosicaoDesejada(distanciaMotorDireito/getDistanciaPorGrau());
		motorEsquerdo->giraParaPosicaoRelativa();
		motorDireito->giraParaPosicaoRelativa();
		esperarAndar();
		motorEsquerdo->setVelocidadeDesejada(velE);
	}

	void circuloParaDireita(double raioCirculo, double grausSemiCirculo){
		//a roda direita deve girar mais rapida
		if(raioCirculo < (distanciaEntreEixos-larguraRodas)){
			raioCirculo = (distanciaEntreEixos-larguraRodas);
		}
		double velD = motorDireito->getVelocidadeDesejada();
		double distanciaMotorDireito = (raioCirculo-(distanciaEntreEixos-larguraRodas))*2*PI*(grausSemiCirculo/360.0);
		double distanciaMotorEsquerdo = (raioCirculo*2*PI)*(grausSemiCirculo/360.0);
		double velocidadeDireito = velD*(distanciaMotorDireito/distanciaMotorEsquerdo);
		motorDireito->setVelocidadeDesejada(velocidadeDireito);
		motorEsquerdo->setPosicaoDesejada(distanciaMotorEsquerdo/getDistanciaPorGrau());
		motorDireito->setPosicaoDesejada(distanciaMotorDireito/getDistanciaPorGrau());
		motorEsquerdo->giraParaPosicaoRelativa();
		motorDireito->giraParaPosicaoRelativa();
		esperarAndar();
		motorDireito->setVelocidadeDesejada(velD);
	}

	//Função que faz o robô girar no próprio eixo
	//a referẽncia é graus positivo gira no sentido horário
	void girarNoEixo(double graus){
		double distanciaMotor = (distanciaEntreEixos-larguraRodas)*PI*(graus/360.0);
		motorDireito->setPosicaoDesejada(distanciaMotor/getDistanciaPorGrau()*-1);
		motorEsquerdo->setPosicaoDesejada(distanciaMotor/getDistanciaPorGrau());
		motorEsquerdo->giraParaPosicaoRelativa();
		motorDireito->giraParaPosicaoRelativa();
		esperarAndar();

	}
	void reset(){
		motorEsquerdo->reset();
		motorDireito->reset();
	}

	void parar(){
		motorEsquerdo->parar();
		motorDireito->parar();
		esperarParar();
	}

	void zerarPosicao(){
		motorEsquerdo->setPosicaoAtual(0);
		motorDireito->setPosicaoAtual(0);
	}

	void esperarParar(){
		while(motorEsquerdo->estaGirando() || motorDireito->estaGirando()){
			usleep(1000);// microsegundos
		}
	}

	//trava a execução do código até o robo chegar na posição determinada pelos parametros
	void esperaChegarNaPosicao(int posicaoEsquerda, int posicaoDireita){
		//a subtração é para tomar decisão antes de chegar exatamente no local
		while(motorEsquerdo->getPosicaoAtual()<=(posicaoEsquerda-15) || motorDireito->getPosicaoAtual()<=(posicaoDireita-15	)){
			//usleep(1000);// microsegundos
		}
	}

	void esperarAndar(){
		while(!motorEsquerdo->estaGirando() && !motorDireito->estaGirando()){
			usleep(1000);// microsegundos
		}
	}
	bool emMovimento(){
		return (motorEsquerdo->estaGirando() || motorDireito->estaGirando());
	}
	void getPosicao(int posicao[2]){
		posicao[0] = motorEsquerdo->getPosicaoAtual();
		posicao[1] = motorDireito->getPosicaoAtual();
		return;
	}

	//grava o movimento do robô por uma quantidade especifica de segundos
	void copiaMovimento(std::string nomeArquivo, int segundos){
		FILE *arq;
		struct timespec start, finish;
		__syscall_slong_t elapsed;
		unsigned long int tempo;
		int posicao[2];
		arq = fopen(nomeArquivo.c_str(), "w+");
		getPosicao(posicao);
		std::cout <<"Aguardando inicio da movimentação do robô...\n";
		while(posicao[0]==0){
			getPosicao(posicao);
		}
		for(int i=0; i <= segundos*5; i++){
			clock_gettime(CLOCK_MONOTONIC, &start);
			getPosicao(posicao);
			fprintf(arq,"%d %d\n",posicao[0],posicao[1]);
			do{
				clock_gettime(CLOCK_MONOTONIC, &finish);
				elapsed  = ((finish.tv_nsec - start.tv_nsec));
				tempo = elapsed / 1000;
				//std::cout << elapsed << std::endl;
			}while(tempo < 200000);//espera 200 milisegundos entre cada leitura

		}
		std::cout<< "gravacao concluida...\n";
		fclose(arq);
	}
	//grava o movimento do robô por uma quantidade especifica de segundos
	bool executaMovimentoCopiado(std::string nomeArquivo){
		FILE *arq;
		int posicao[2];
		int posicaoAntiga[2];
		int velocidadeEsquerda, velocidadeDireita, velocidadeInstantaneaEsquerda, velocidadeInstantaneaDireita;
		velocidadeEsquerda = motorEsquerdo->getVelocidadeDesejada();
		velocidadeDireita = motorDireito->getVelocidadeDesejada();
		arq = fopen(nomeArquivo.c_str(), "r");
		if(!arq){
			std::cout << "erro, arquivo não encontrado...\n";
			return false;
		}
		zerarPosicao();
		posicaoAntiga[0]=posicaoAntiga[1]=0;
		fscanf(arq,"%d %d\n",&posicao[0],&posicao[1]); //descarta a primeira leitura
		while(!feof(arq)){
			fscanf(arq,"%d %d\n",&posicao[0],&posicao[1]);
			if(posicaoAntiga[0]==posicao[0] && posicaoAntiga[1]==posicao[1] ){
				continue; //se o proximo valor for igual o anterior, pulo ele
			}
			//se o lado esquerdo anda mais que o lado direito, diminui a velocidade do lado direito
			std::cout <<" distancia por tempo: " <<  posicao[0]-posicaoAntiga[0] <<" "<< posicao[1]-posicaoAntiga[1] << std::endl;
			if(posicao[0]-posicaoAntiga[0] > posicao[1]-posicaoAntiga[1]){
				velocidadeInstantaneaDireita = ((posicao[1]-posicaoAntiga[1])/((posicao[0]-posicaoAntiga[0])*1.0))*(velocidadeDireita*1.0);
				velocidadeInstantaneaEsquerda = velocidadeEsquerda;
			}else if(posicao[0]-posicaoAntiga[0] < posicao[1]-posicaoAntiga[1]){ //caso a posicao direita ande mais...
				velocidadeInstantaneaEsquerda = ((posicao[0]-posicaoAntiga[0])/((posicao[1]-posicaoAntiga[1])*1.0))*(velocidadeEsquerda*1.0);
				velocidadeInstantaneaDireita = velocidadeDireita;
			}else{
				velocidadeInstantaneaDireita = velocidadeDireita;
				velocidadeInstantaneaEsquerda = velocidadeEsquerda;
			}
			//velocidadeInstantaneaDireita = (posicao[1]-posicaoAntiga[1])*5;
			//velocidadeInstantaneaEsquerda = (posicao[0]-posicaoAntiga[0])*5;
			std::cout <<" Velocidades instantaneas: " << velocidadeInstantaneaEsquerda << " " << velocidadeInstantaneaDireita << std::endl;
			motorDireito->setVelocidadeDesejada(velocidadeInstantaneaDireita);
			motorEsquerdo->setVelocidadeDesejada(velocidadeInstantaneaEsquerda);
			andarAteValorAbsoluto(posicao[0],posicao[1]);
			posicaoAntiga[0] = posicao[0];
			posicaoAntiga[1] = posicao[1];
			esperaChegarNaPosicao(posicaoAntiga[0],posicaoAntiga[1]);
		}
		//esperarParar();
		return true;
	}

};


#endif /* MOVIMENTO_H_ */
