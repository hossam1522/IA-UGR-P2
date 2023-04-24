#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>

bool AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);
list<Action> AnchuraSoloJugador_V2(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);
list<Action> AnchuraSoloJugador_V3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);
bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char> > &mapa);
ubicacion NextCasilla(const ubicacion &pos);
stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char> > &mapa);
bool Find(const stateN0 &item, const list<stateN0> &lista);
bool Find(const stateN0 &item, const list<nodeN0> &lista);
void AnularMatriz(vector<vector<unsigned char>> &matriz);

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;

	if (sensores.nivel != 4){
		if (!hayPlan){
			// Invocar al método de búsqueda
			cout << "Calculando un nuevo plan..." << endl;
			c_state.jugador.f = sensores.posF;
			c_state.jugador.c = sensores.posC;
			c_state.jugador.brujula = sensores.sentido;
			c_state.sonambulo.f = sensores.SONposF;
			c_state.sonambulo.c = sensores.SONposC;
			c_state.sonambulo.brujula = sensores.SONsentido;
			goal.f = sensores.destinoF;
			goal.c = sensores.destinoC;

			switch (sensores.nivel){
				case 0:
					plan = AnchuraSoloJugador_V3(c_state, goal, mapaResultado);
					break;
				case 1:
					cout << "Nivel 1 no implementado" << endl;
					break;
				case 2:
					cout << "Nivel 2 no implementado" << endl;
					break;
				case 3:
					cout << "Nivel 3 no implementado" << endl;
					break;
			}
		}
		if (hayPlan and plan.size()>0){
			cout << "Ejecutando la siguiente acción del plan" << endl;
			accion = plan.front();
			plan.pop_front();
		}
		if (plan.size()== 0){
			cout << "Se completó el plan" << endl;
			hayPlan = false;
		}

	}
	else{
		cout << "Nivel 4 no implementado" << endl;
	}

	return accion;
}


int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}

bool AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){

	stateN0 current_state = inicio;
	list<stateN0> frontier;
	list<stateN0> explored;
	frontier.push_back(current_state);
	bool SolutionFound = (current_state.jugador.f == final.f && current_state.jugador.c == final.c);

	while (!frontier.empty() && !SolutionFound){
		frontier.pop_front();
		explored.push_back(current_state);

		// Generar hijo actFORWARD
		stateN0 child_forward = apply(actFORWARD, current_state, mapa);
		if (child_forward.jugador.f == final.f && child_forward.jugador.c == final.c){
			current_state = child_forward;
			SolutionFound = true;
		}
		else if (!Find(child_forward, frontier) && !Find(child_forward, explored))
			frontier.push_back(child_forward);

		if (!SolutionFound){
			// Generar hijo actTURN_L
			stateN0 child_turnl = apply(actTURN_L, current_state, mapa);
			if (!Find(child_turnl, frontier) && !Find(child_turnl, explored))
				frontier.push_back(child_turnl);

			// Generar hijo actTURN_R
			stateN0 child_turnr = apply(actTURN_R, current_state, mapa);
			if (!Find(child_turnr, frontier) && !Find(child_turnr, explored))
				frontier.push_back(child_turnr);
		}

		if (!SolutionFound && !frontier.empty())
			current_state = frontier.front();

	}

	return SolutionFound;
}

list<Action> AnchuraSoloJugador_V2(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){
	nodeN0 current_node;
	list<nodeN0> frontier;
	list<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;
	bool SolutionFound = (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() && !SolutionFound){
		frontier.pop_front();
		explored.push_back(current_node);

		// Generar hijo actFORWARD
		nodeN0 child_forward = current_node;
		child_forward.st = apply(actFORWARD, current_node.st, mapa);
		if (child_forward.st.jugador.f == final.f && child_forward.st.jugador.c == final.c){
			child_forward.secuencia.push_back(actFORWARD);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (!Find(child_forward.st, frontier) && !Find(child_forward.st, explored)){
			child_forward.secuencia.push_back(actFORWARD);
			frontier.push_back(child_forward);
		}

		if (!SolutionFound){
			// Generar hijo actTURN_L
			nodeN0 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (!Find(child_turnl.st, frontier) && !Find(child_turnl.st, explored)){
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN0 child_turnr = current_node;
			child_turnr.st = apply(actTURN_L, current_node.st, mapa);
			if (!Find(child_turnr.st, frontier) && !Find(child_turnr.st, explored)){
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty())
			current_node = frontier.front();
	}

	if (SolutionFound){
		plan = current_node.secuencia;
	}

	return plan;
}

list<Action> AnchuraSoloJugador_V3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){
	nodeN0 current_node;
	list<nodeN0> frontier;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;
	bool SolutionFound = (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() && !SolutionFound){
		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actFORWARD
		nodeN0 child_forward = current_node;
		child_forward.st = apply(actFORWARD, current_node.st, mapa);
		if (child_forward.st.jugador.f == final.f && child_forward.st.jugador.c == final.c){
			child_forward.secuencia.push_back(actFORWARD);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end()){
			child_forward.secuencia.push_back(actFORWARD);
			frontier.push_back(child_forward);
		}

		if (!SolutionFound){
			// Generar hijo actTURN_L
			nodeN0 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end()){
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN0 child_turnr = current_node;
			child_turnr.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end()){
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty()){
			current_node = frontier.front();
			while (!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop_front();
				current_node = frontier.front();
			}
		}
	}

	if (SolutionFound){
		plan = current_node.secuencia;
	}

	return plan;
}

bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char> > &mapa){
	return (mapa[x.f][x.c] != 'P' && mapa[x.f][x.c] != 'M');
}

ubicacion NextCasilla(const ubicacion &pos){
	ubicacion next = pos;
	switch (pos.brujula){
		case norte:
			next.f = pos.f - 1;
			break;
		case noreste:
			next.f = pos.f - 1;
			next.c = pos.c + 1;
			break;
		case este:
			next.c = pos.c + 1;
			break;
		case sureste:
			next.f = pos.f + 1;
			next.c = pos.c + 1;
			break;
		case sur:
			next.f = pos.f + 1;
			break;
		case suroeste:
			next.f = pos.f + 1;
			next.c = pos.c - 1;
			break;
		case oeste:
			next.c = pos.c - 1;
			break;
		case noroeste:
			next.f = pos.f - 1;
			next.c = pos.c - 1;
			break;
		default:
			break;
	}

	return next;
}

stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char> > &mapa){
	stateN0 st_result = st;
	ubicacion sig_ubicacion;
	switch (a){
		case actFORWARD:
			sig_ubicacion = NextCasilla(st.jugador);
			if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == st.sonambulo.f && sig_ubicacion.c == st.sonambulo.c)){
				st_result.jugador = sig_ubicacion;
			}
			break;
		case actTURN_L:
			st_result.jugador.brujula = static_cast<Orientacion>((st.jugador.brujula + 6) % 8);
			break;
		case actTURN_R:
			st_result.jugador.brujula = static_cast<Orientacion>((st.jugador.brujula + 2) % 8);
			break;
	}
	return st_result;
}

bool Find(const stateN0 &item, const list<stateN0> &lista){
	auto it = lista.begin();
	while (it != lista.end() && !(*it == item))
		it++;

	return (!(it == lista.end()));
}

bool Find(const stateN0 &item, const list<nodeN0> &lista){
	auto it = lista.begin();
	while (it != lista.end() && !(it->st == item))
		it++;

	return (!(it == lista.end()));
}

void AnularMatriz(vector<vector<unsigned char>> &matriz){
	for (int i = 0; i < matriz.size(); i++){
		for (int j = 0; j < matriz[i].size(); j++){
			matriz[i][j] = 0;
		}
	}
}

void ComportamientoJugador::VisualizaPlan(const stateN0 &st, const list<Action> &plan){
	AnularMatriz(mapaConPlan);
	stateN0 cst = st;

	auto it = plan.begin();
	while (it != plan.end()){
		switch (*it){
			case actFORWARD:
				cst.jugador = NextCasilla(cst.jugador);
				mapaConPlan[cst.jugador.f][cst.jugador.c] = 1;
				break;
			case actTURN_R:
				cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 2) % 8);
				break;
			case actTURN_L:
				cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 6) % 8);
				break;
			case actSON_FORWARD:
				cst.sonambulo = NextCasilla(cst.sonambulo);
				mapaConPlan[cst.sonambulo.f][cst.sonambulo.c] = 2;
				break;
			case actSON_TURN_SR:
				cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 1) % 8);
				break;
			case actSON_TURN_SL:
				cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 7) % 8);
				break;
		}
		it++;
	}
}
