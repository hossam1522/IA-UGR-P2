#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>
#include <cmath>
#include <bits/stdc++.h>


/* ..................................Declaración nivel 0................................................ */
list<Action> AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);
bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char> > &mapa);
ubicacion NextCasilla(const ubicacion &pos);
stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char> > &mapa);
void AnularMatriz(vector<vector<unsigned char>> &matriz);

/* ..................................Declaración nivel 1................................................ */
bool SON_aLaVista(const stateN0 &st);
list<Action> AnchuraAmbos(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);

/* ..................................Declaración nivel 2................................................ */
nodeN2 apply(const Action &a, const nodeN2 &n, const vector<vector<unsigned char> > &mapa);
struct ComparaCosteN2{
	bool operator()(const nodeN2 &n1, const nodeN2 &n2) const{
		return n1.coste > n2.coste;
	}
};
list<Action> DijkstraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);

/* ..................................Declaración nivel 3................................................ */
nodeN3 apply(const Action &a, const nodeN3 &n, const vector<vector<unsigned char> > &mapa);
struct ComparaCosteN3{
	bool operator()(const nodeN3 &n1, const nodeN3 &n2) const{
		return ((n1.coste+n1.heuristica) > (n2.coste+n2.heuristica));
	}
};
int distanciaManhattan(const ubicacion &a, const ubicacion &b);
int distanciaEuclidea(const ubicacion &a, const ubicacion &b);
int distanciaChebyshev(const ubicacion &a, const ubicacion &b);
int aplicarHeurisitica (const nodeN3 &n, const ubicacion &final);
list<Action> AEstrellaAmbos(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);

/* ..................................Declaración nivel 4................................................ */

bool CasillaTransitableN4(const ubicacion &x, const vector<vector<unsigned char> > &mapa);
stateN4 apply(const Action &a, const stateN4 &st, const vector<vector<unsigned char> > &mapa);
list <Action> AnchuraSoloJugadorN4 (const stateN4 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa);



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
					plan = AnchuraSoloJugador(c_state, goal, mapaResultado);
					break;
				case 1:
					plan = AnchuraAmbos(c_state, goal, mapaResultado);
					break;
				case 2:
					plan = DijkstraSoloJugador(c_state, goal, mapaResultado);
					break;
				case 3:
					plan = AEstrellaAmbos(c_state, goal, mapaResultado);
					break;
			}
			if (plan.size() > 0){
				VisualizaPlan(c_state, plan);
				hayPlan = true;
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

		/* if (sensores.reset){
			bien_situado = false;
			ejecutadoWHEREIS = false;
		}

		if (!bien_situado){
			accion = actWHEREIS;
			bien_situado = true;

		}
		else {

			if (!ejecutadoWHEREIS){
				c_state_N4.jugador.f=sensores.posF;
				c_state_N4.jugador.c=sensores.posC;
				c_state_N4.jugador.brujula=sensores.sentido;
				c_state_N4.sonambulo.f=sensores.SONposF;
				c_state_N4.sonambulo.c=sensores.SONposC;
				c_state_N4.sonambulo.brujula=sensores.SONsentido;
				ejecutadoWHEREIS = true;
				plan = list<Action>();
				plan.push_back(actFORWARD);
				plan.push_back(actTURN_L);
				plan.push_back(actFORWARD);
				plan.push_back(actTURN_L);
				plan.push_back(actFORWARD);
				plan.push_back(actFORWARD);
				hayPlan = true;
			}
			else{

				actualizarVariablesEstado();
				rellenarMapa(sensores, mapaResultado);

				if (loboCerca(sensores)){
					accion = actIDLE;
				}
				else if (!hayPlan){

					// Invocar al método de búsqueda
					cout << "Calculando un nuevo plan..." << endl;
					goal.f = sensores.destinoF;
					goal.c = sensores.destinoC;

					plan = AnchuraSoloJugadorN4(c_state_N4, goal, mapaResultado);


					if (plan.size() > 0){
						VisualizaPlan(c_state_N4, plan);
						hayPlan = true;
					}
				}
				else if (hayPlan and plan.size()>0){
					cout << "Ejecutando la siguiente acción del plan" << endl;
					accion = plan.front();
					plan.pop_front();
				}
				else if (plan.size()== 0){
					cout << "Se completó el plan" << endl;
					hayPlan = false;
				}

			}

		} */

		//Action accion;

		if (!bien_situado){
			accion = actWHEREIS;
			bien_situado = true;

		} 
		else if (!ejecutadoWHEREIS){
			c_state_N4.jugador.f=sensores.posF;
			c_state_N4.jugador.c=sensores.posC;
			c_state_N4.jugador.brujula=sensores.sentido;
			c_state_N4.sonambulo.f=sensores.SONposF;
			c_state_N4.sonambulo.c=sensores.SONposC;
			c_state_N4.sonambulo.brujula=sensores.SONsentido;
			ejecutadoWHEREIS = true;
		} 
		else {

			if (sensores.reset){
				bien_situado = false;
				c_state_N4.tiene_bikini = false;
				c_state_N4.tiene_zapatillas = false;
			}

			actualizarVariablesEstado();


			if (sensores.terreno[0] == 'K' && !c_state_N4.tiene_bikini){
				c_state_N4.tiene_bikini = true;
			}

			if (sensores.terreno[0] == 'D' && !c_state_N4.tiene_zapatillas){
				c_state_N4.tiene_zapatillas = true;
			}

			if (sensores.terreno[0] == 'X' && !recargando){
				recargando = true;
			}

			mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c]++;
			rellenarMapa(sensores, mapaResultado);

			if(!precipicios_rellenos){
				rellenarPrecipicios(mapaResultado);
				precipicios_rellenos=true;
			}

			//Decidir la nueva acción
			elegirMovimiento(accion, sensores, c_state_N4);

		}

	}

	last_action = accion;
	return accion;
}

/* ..................................Implementación nivel 0................................................ */

int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}

list<Action> AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){
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
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end()){
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty()){
			current_node = frontier.front();
			while (!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop_front();
				if (!frontier.empty())
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
	return (mapa[x.f][x.c] != 'P' && mapa[x.f][x.c] != 'M' /* && mapa[x.f][x.c]!='?' */);
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
		case actSON_FORWARD:
			sig_ubicacion = NextCasilla(st.sonambulo);
			if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == st.jugador.f && sig_ubicacion.c == st.jugador.c)){
				st_result.sonambulo = sig_ubicacion;
			}
			break;
		case actSON_TURN_SL:
			st_result.sonambulo.brujula = static_cast<Orientacion>((st.sonambulo.brujula + 7) % 8);
			break;
		case actSON_TURN_SR:
			st_result.sonambulo.brujula = static_cast<Orientacion>((st.sonambulo.brujula + 1) % 8);
			break;
	}
	return st_result;
}

void AnularMatriz(vector<vector<unsigned char>> &matriz){
	for (int i = 0; i < matriz.size(); i++){
		for (int j = 0; j < matriz[i].size(); j++){
			matriz[i][j] = 0;
		}
	}
}


/* ..................................Implementación nivel 1................................................ */

bool SON_aLaVista(const stateN0 &st){
	bool aLaVista = false;
	ubicacion pos = st.jugador;

	switch (pos.brujula){
		case norte:
		  if (pos.f - st.sonambulo.f == 1 && abs(st.sonambulo.c - pos.c) <= 1)
				aLaVista = true;
			else if (pos.f - st.sonambulo.f == 2 && abs(st.sonambulo.c - pos.c) <= 2)
				aLaVista = true;
			else if (pos.f - st.sonambulo.f == 3 && abs(st.sonambulo.c - pos.c) <= 3)
				aLaVista = true;
			break;
		case noreste:
			if (pos.f - st.sonambulo.f <=3 && st.sonambulo.c - pos.c <= 3 && pos.f - st.sonambulo.f >= 1 && st.sonambulo.c - pos.c >= 1)
				aLaVista = true;
			break;
		case este:
			if (st.sonambulo.c - pos.c == 1 && abs(st.sonambulo.f - pos.f) <= 1)
				aLaVista = true;
			else if (st.sonambulo.c - pos.c == 2 && abs(st.sonambulo.f - pos.f) <= 2)
				aLaVista = true;
			else if (st.sonambulo.c - pos.c == 3 && abs(st.sonambulo.f - pos.f) <= 3)
				aLaVista = true;
			break;
		case sureste:
			if (st.sonambulo.f - pos.f <= 3 && st.sonambulo.f - pos.f >= 1 && st.sonambulo.c - pos.c <= 3 && st.sonambulo.c - pos.c >= 1)
				aLaVista = true;
			break;
		case sur:
			if (st.sonambulo.f - pos.f == 1 && abs(st.sonambulo.c - pos.c) <= 1)
				aLaVista = true;
			else if (st.sonambulo.f - pos.f == 2 && abs(st.sonambulo.c - pos.c) <= 2)
				aLaVista = true;
			else if (st.sonambulo.f - pos.f == 3 && abs(st.sonambulo.c - pos.c) <= 3)
				aLaVista = true;
			break;
		case suroeste:
			if (st.sonambulo.f - pos.f <= 3 && st.sonambulo.f - pos.f >= 1 && pos.c - st.sonambulo.c <= 3 && pos.c - st.sonambulo.c >= 1)
				aLaVista = true;
			break;
		case oeste:
			if (pos.c - st.sonambulo.c == 1 && abs(st.sonambulo.f - pos.f) <=1)
				aLaVista = true;
			else if (pos.c - st.sonambulo.c == 2 && abs(st.sonambulo.f - pos.f) <= 2)
				aLaVista = true;
			else if (pos.c - st.sonambulo.c == 3 && abs(st.sonambulo.f - pos.f) <= 3)
				aLaVista = true;
			break;
		case noroeste:
			if (pos.f - st.sonambulo.f <= 3 && pos.f - st.sonambulo.f >= 1 && pos.c - st.sonambulo.c <= 3 && pos.c - st.sonambulo.c >= 1)
				aLaVista = true;
			break;
	}

	return aLaVista;
}

list<Action> AnchuraAmbos(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){
	nodeN1 current_node;
	list<nodeN1> frontier;
	set<nodeN1> explored;
	list<Action> plan;
	current_node.st = inicio;
	bool SolutionFound = (current_node.st.sonambulo.f == final.f && current_node.st.sonambulo.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() && !SolutionFound){
		frontier.pop_front();
		explored.insert(current_node);

		if (SON_aLaVista(current_node.st)){

			// Generar hijo actSON_FORWARD
			nodeN1 child_SON_forward = current_node;
			child_SON_forward.st = apply(actSON_FORWARD, current_node.st, mapa);
			if (child_SON_forward.st.sonambulo.f == final.f && child_SON_forward.st.sonambulo.c == final.c){
				child_SON_forward.secuencia.push_back(actSON_FORWARD);
				current_node = child_SON_forward;
				SolutionFound = true;
			}
			else if (explored.find(child_SON_forward) == explored.end()){
				child_SON_forward.secuencia.push_back(actSON_FORWARD);
				frontier.push_back(child_SON_forward);
			}
		}

		if (!SolutionFound){

			if (SON_aLaVista(current_node.st)){

				// Generar hijo actSON_TURN_SL
				nodeN1 child_SON_turnsl = current_node;
				child_SON_turnsl.st = apply(actSON_TURN_SL, current_node.st, mapa);
				if (explored.find(child_SON_turnsl) == explored.end()){
					child_SON_turnsl.secuencia.push_back(actSON_TURN_SL);
					frontier.push_back(child_SON_turnsl);
				}

				// Generar hijo actSON_TURN_SR
				nodeN1 child_SON_turnsr = current_node;
				child_SON_turnsr.st = apply(actSON_TURN_SR, current_node.st, mapa);
				if (explored.find(child_SON_turnsr) == explored.end()){
					child_SON_turnsr.secuencia.push_back(actSON_TURN_SR);
					frontier.push_back(child_SON_turnsr);
				}

			}

			// Generar hijo actFORWARD
			nodeN1 child_forward = current_node;
			child_forward.st = apply(actFORWARD, current_node.st, mapa);
			if (explored.find(child_forward) == explored.end()){
				child_forward.secuencia.push_back(actFORWARD);
				frontier.push_back(child_forward);
			}

			// Generar hijo actTURN_L
			nodeN1 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end()){
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN1 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end()){
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}

		}

		if (!SolutionFound && !frontier.empty()){
			current_node = frontier.front();
			while (!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}

	if (SolutionFound){
		plan = current_node.secuencia;
	}
	else {
		cout << "No se ha encontrado solución" << endl;
	}

	return plan;
}


/* ..................................Implementación nivel 2................................................ */

nodeN2 apply(const Action &a, const nodeN2 &n, const vector<vector<unsigned char> > &mapa){
	nodeN2 n_result = n;
	ubicacion sig_ubicacion;
	switch (a){
		case actFORWARD:
			sig_ubicacion = NextCasilla(n.n.st.jugador);
			if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == n.n.st.sonambulo.f && sig_ubicacion.c == n.n.st.sonambulo.c)){
				n_result.n.st.jugador = sig_ubicacion;
				if (mapa[sig_ubicacion.f][sig_ubicacion.c] == 'K' && !n.tiene_bikini){
					n_result.tiene_bikini = true;
					n_result.tiene_zapatillas = false;
				}
				if (mapa[sig_ubicacion.f][sig_ubicacion.c] == 'D' && !n.tiene_zapatillas){
					n_result.tiene_zapatillas = true;
					n_result.tiene_bikini = false;
				}

				if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'A')
					if (n.tiene_bikini)
						n_result.coste += 10;
					else
						n_result.coste += 100;
				else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'B')
					if (n.tiene_zapatillas)
						n_result.coste += 15;
					else
						n_result.coste += 50;
				else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'T')
					n_result.coste += 2;
				else
					n_result.coste += 1;
			}

			break;

		case actTURN_L:
			n_result.n.st.jugador.brujula = static_cast<Orientacion>((n.n.st.jugador.brujula + 6) % 8);
			if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'A')
				if (n.tiene_bikini)
					n_result.coste += 5;
				else
					n_result.coste += 25;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'B')
				if (n.tiene_zapatillas)
					n_result.coste += 1;
				else
					n_result.coste += 5;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'T')
				n_result.coste += 2;
			else
				n_result.coste += 1;
			break;

		case actTURN_R:
			n_result.n.st.jugador.brujula = static_cast<Orientacion>((n.n.st.jugador.brujula + 2) % 8);
			if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'A')
				if (n.tiene_bikini)
					n_result.coste += 5;
				else
					n_result.coste += 25;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'B')
				if (n.tiene_zapatillas)
					n_result.coste += 1;
				else
					n_result.coste += 5;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'T')
				n_result.coste += 2;
			else
				n_result.coste += 1;
			break;

		case actSON_FORWARD:
			sig_ubicacion = NextCasilla(n.n.st.sonambulo);
			if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == n.n.st.jugador.f && sig_ubicacion.c == n.n.st.jugador.c))
				n_result.n.st.sonambulo = sig_ubicacion;
			break;

		case actSON_TURN_SL:
			n_result.n.st.sonambulo.brujula = static_cast<Orientacion>((n.n.st.sonambulo.brujula + 7) % 8);
			break;

		case actSON_TURN_SR:
			n_result.n.st.sonambulo.brujula = static_cast<Orientacion>((n.n.st.sonambulo.brujula + 1) % 8);
			break;
	}
	return n_result;
}

list<Action> DijkstraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){
	nodeN2 current_node;
	priority_queue<nodeN2, vector<nodeN2>, ComparaCosteN2> frontier;
	set<nodeN2> explored;
	list<Action> plan;
	current_node.n.st = inicio;
	if (mapa[inicio.jugador.f][inicio.jugador.c] == 'K'){
		current_node.tiene_bikini = true;
		current_node.tiene_zapatillas = false;
	}
	else if (mapa[inicio.jugador.f][inicio.jugador.c] == 'D'){
		current_node.tiene_zapatillas = true;
		current_node.tiene_bikini = false;
	}
	bool SolutionFound = (current_node.n.st.jugador.f == final.f && current_node.n.st.jugador.c == final.c);
	frontier.push(current_node);

	while (!frontier.empty() && !SolutionFound){
		frontier.pop();
		explored.insert(current_node);

		if (current_node.n.st.jugador.f == final.f && current_node.n.st.jugador.c == final.c){
			SolutionFound = true;
			plan = current_node.n.secuencia;
		}

		if (!SolutionFound){

			// Generar hijo actFORWARD
			nodeN2 child_forward = current_node;
			child_forward = apply(actFORWARD, current_node, mapa);
			if (explored.find(child_forward) == explored.end()){
				child_forward.n.secuencia.push_back(actFORWARD);
				frontier.push(child_forward);
			}

			// Generar hijo actTURN_L
			nodeN2 child_turnl = current_node;
			child_turnl = apply(actTURN_L, current_node, mapa);
			if (explored.find(child_turnl) == explored.end()){
				child_turnl.n.secuencia.push_back(actTURN_L);
				frontier.push(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN2 child_turnr = current_node;
			child_turnr = apply(actTURN_R, current_node, mapa);
			if (explored.find(child_turnr) == explored.end()){
				child_turnr.n.secuencia.push_back(actTURN_R);
				frontier.push(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty()){
			current_node = frontier.top();
			while (!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop();
				if (!frontier.empty())
					current_node = frontier.top();

			}
		}
	}

	return plan;
}

/* ..................................Implementación nivel 3................................................ */

int distanciaManhattan(const ubicacion &a, const ubicacion &b){
	return abs(a.f - b.f) + abs(a.c - b.c);
}

int distanciaEuclidea(const ubicacion &a, const ubicacion &b){
	return sqrt(pow(a.f - b.f, 2) + pow(a.c - b.c, 2));
}

int distanciaChebyshev(const ubicacion &a, const ubicacion &b){
	return max(abs(a.f - b.f), abs(a.c - b.c));
}

int aplicarHeurisitica (const nodeN3 &n, const ubicacion &final){
	return (distanciaEuclidea(n.n.st.sonambulo, final));
	//return (distanciaEuclidea(n.n.st.jugador, final) + distanciaEuclidea(n.n.st.sonambulo, final));
	//return (distanciaChebyshev(n.n.st.jugador, final) + distanciaChebyshev(n.n.st.sonambulo, final));
	//return (distanciaChebyshev(n.n.st.sonambulo, final));
	//return distanciaManhattan(n.n.st.sonambulo, final);
	//return distanciaManhattan(n.n.st.jugador, final) + distanciaManhattan(n.n.st.sonambulo, final);

}

nodeN3 apply(const Action &a, const nodeN3 &n, const vector<vector<unsigned char> > &mapa){
	nodeN3 n_result = n;
	ubicacion sig_ubicacion;
	switch (a){
		case actFORWARD:
			sig_ubicacion = NextCasilla(n.n.st.jugador);
			if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == n.n.st.sonambulo.f && sig_ubicacion.c == n.n.st.sonambulo.c)){
				n_result.n.st.jugador = sig_ubicacion;
				if (mapa[sig_ubicacion.f][sig_ubicacion.c] == 'K' && !n.tiene_bikini_J){
					n_result.tiene_bikini_J = true;
					n_result.tiene_zapatillas_J = false;
				}
				if (mapa[sig_ubicacion.f][sig_ubicacion.c] == 'D' && !n.tiene_zapatillas_J){
					n_result.tiene_zapatillas_J = true;
					n_result.tiene_bikini_J = false;
				}

				if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'A')
					if (n.tiene_bikini_J)
						n_result.coste += 10;
					else
						n_result.coste += 100;
				else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'B')
					if (n.tiene_zapatillas_J)
						n_result.coste += 15;
					else
						n_result.coste += 50;
				else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'T')
					n_result.coste += 2;
				else
					n_result.coste += 1;
			}

			break;

		case actTURN_L:
			n_result.n.st.jugador.brujula = static_cast<Orientacion>((n.n.st.jugador.brujula + 6) % 8);
			if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'A')
				if (n.tiene_bikini_J)
					n_result.coste += 5;
				else
					n_result.coste += 25;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'B')
				if (n.tiene_zapatillas_J)
					n_result.coste += 1;
				else
					n_result.coste += 5;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'T')
				n_result.coste += 2;
			else
				n_result.coste += 1;
			break;

		case actTURN_R:
			n_result.n.st.jugador.brujula = static_cast<Orientacion>((n.n.st.jugador.brujula + 2) % 8);
			if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'A')
				if (n.tiene_bikini_J)
					n_result.coste += 5;
				else
					n_result.coste += 25;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'B')
				if (n.tiene_zapatillas_J)
					n_result.coste += 1;
				else
					n_result.coste += 5;
			else if (mapa[n.n.st.jugador.f][n.n.st.jugador.c] == 'T')
				n_result.coste += 2;
			else
				n_result.coste += 1;
			break;

		case actSON_FORWARD:
			sig_ubicacion = NextCasilla(n.n.st.sonambulo);
			if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == n.n.st.jugador.f && sig_ubicacion.c == n.n.st.jugador.c)){
				n_result.n.st.sonambulo = sig_ubicacion;
				if (mapa[sig_ubicacion.f][sig_ubicacion.c] == 'K' && !n.tiene_bikini_SON){
					n_result.tiene_bikini_SON = true;
					n_result.tiene_zapatillas_SON = false;
				}
				if (mapa[sig_ubicacion.f][sig_ubicacion.c] == 'D' && !n.tiene_zapatillas_SON){
					n_result.tiene_zapatillas_SON = true;
					n_result.tiene_bikini_SON = false;
				}

				if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'A')
					if (n.tiene_bikini_SON)
						n_result.coste += 10;
					else
						n_result.coste += 100;
				else if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'B')
					if (n.tiene_zapatillas_SON)
						n_result.coste += 15;
					else
						n_result.coste += 50;
				else if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'T')
					n_result.coste += 2;
				else
					n_result.coste += 1;
			}
			break;

		case actSON_TURN_SL:
			n_result.n.st.sonambulo.brujula = static_cast<Orientacion>((n.n.st.sonambulo.brujula + 7) % 8);
			if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'A')
				if (n.tiene_bikini_SON)
					n_result.coste += 2;
				else
					n_result.coste += 7;
			else if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'B')
				if (n.tiene_zapatillas_SON)
					n_result.coste += 1;
				else
					n_result.coste += 3;
			else if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'T')
				n_result.coste += 1;
			else
				n_result.coste += 1;
			break;

		case actSON_TURN_SR:
			n_result.n.st.sonambulo.brujula = static_cast<Orientacion>((n.n.st.sonambulo.brujula + 1) % 8);
			if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'A')
				if (n.tiene_bikini_SON)
					n_result.coste += 2;
				else
					n_result.coste += 7;
			else if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'B')
				if (n.tiene_zapatillas_SON)
					n_result.coste += 1;
				else
					n_result.coste += 3;
			else if (mapa[n.n.st.sonambulo.f][n.n.st.sonambulo.c] == 'T')
				n_result.coste += 1;
			else
				n_result.coste += 1;
			break;
	}
	return n_result;
}

list<Action> AEstrellaAmbos(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){
	nodeN3 current_node;
	priority_queue<nodeN3, vector<nodeN3>, ComparaCosteN3> frontier;
	set<nodeN3> explored;
	list<Action> plan;
	current_node.n.st = inicio;
	if (mapa[inicio.jugador.f][inicio.jugador.c] == 'K'){
		current_node.tiene_bikini_J = true;
		current_node.tiene_zapatillas_J = false;
	}
	else if (mapa[inicio.jugador.f][inicio.jugador.c] == 'D'){
		current_node.tiene_zapatillas_J = true;
		current_node.tiene_bikini_J = false;
	}
	if (mapa[inicio.sonambulo.f][inicio.sonambulo.c] == 'K'){
		current_node.tiene_bikini_SON = true;
		current_node.tiene_zapatillas_SON = false;
	}
	else if (mapa[inicio.sonambulo.f][inicio.sonambulo.c] == 'D'){
		current_node.tiene_zapatillas_SON = true;
		current_node.tiene_bikini_SON = false;
	}
	current_node.heuristica = aplicarHeurisitica(current_node, final);
	bool SolutionFound = (current_node.n.st.sonambulo.f == final.f && current_node.n.st.sonambulo.c == final.c);
	frontier.push(current_node);

	while (!frontier.empty() && !SolutionFound){
		frontier.pop();
		explored.insert(current_node);

		if (current_node.n.st.sonambulo.f == final.f && current_node.n.st.sonambulo.c == final.c){
			SolutionFound = true;
			plan = current_node.n.secuencia;
		}

		if (!SolutionFound){

			if (SON_aLaVista(current_node.n.st)){

				// Generar hijo actSON_FORWARD
				nodeN3 child_SON_forward = current_node;
				child_SON_forward = apply(actSON_FORWARD, current_node, mapa);
				child_SON_forward.heuristica = aplicarHeurisitica(child_SON_forward, final);
				if (explored.find(child_SON_forward) == explored.end()){
					child_SON_forward.n.secuencia.push_back(actSON_FORWARD);
					frontier.push(child_SON_forward);
				}

				// Generar hijo actSON_TURN_SL
				nodeN3 child_SON_turnsl = current_node;
				child_SON_turnsl = apply(actSON_TURN_SL, current_node, mapa);
				child_SON_turnsl.heuristica = aplicarHeurisitica(child_SON_turnsl, final);
				if (explored.find(child_SON_turnsl) == explored.end() ){
					child_SON_turnsl.n.secuencia.push_back(actSON_TURN_SL);
					frontier.push(child_SON_turnsl);
				}

				// Generar hijo actSON_TURN_SR
				nodeN3 child_SON_turnsr = current_node;
				child_SON_turnsr = apply(actSON_TURN_SR, current_node, mapa);
				child_SON_turnsr.heuristica = aplicarHeurisitica(child_SON_turnsr, final);
				if (explored.find(child_SON_turnsr) == explored.end() ){
					child_SON_turnsr.n.secuencia.push_back(actSON_TURN_SR);
					frontier.push(child_SON_turnsr);
				}

			}

			// Generar hijo actFORWARD
			nodeN3 child_forward = current_node;
			child_forward = apply(actFORWARD, current_node, mapa);
			child_forward.heuristica = aplicarHeurisitica(child_forward, final);
			if (explored.find(child_forward) == explored.end()){
				child_forward.n.secuencia.push_back(actFORWARD);
				frontier.push(child_forward);
			}

			// Generar hijo actTURN_L
			nodeN3 child_turnl = current_node;
			child_turnl = apply(actTURN_L, current_node, mapa);
			child_turnl.heuristica = aplicarHeurisitica(child_turnl, final);
			if (explored.find(child_turnl) == explored.end()){
				child_turnl.n.secuencia.push_back(actTURN_L);
				frontier.push(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN3 child_turnr = current_node;
			child_turnr = apply(actTURN_R, current_node, mapa);
			child_turnr.heuristica = aplicarHeurisitica(child_turnr, final);
			if (explored.find(child_turnr) == explored.end() ){
				child_turnr.n.secuencia.push_back(actTURN_R);
				frontier.push(child_turnr);
			}

		}

		if (!SolutionFound && !frontier.empty()){
			current_node = frontier.top();
			while (!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop();
				if (!frontier.empty())
					current_node = frontier.top();
			}
		}
	}

	if (!SolutionFound) {
		cout << "No se ha encontrado solución" << endl;
	}

	return plan;
}

/* ..................................Implementación nivel 4................................................ */

void ComportamientoJugador::actualizarVariablesEstado(){
	int a;
	// Actualizacion de las variables de estado
	switch (last_action){
		case actFORWARD:
			switch (c_state_N4.jugador.brujula){
				case norte: c_state_N4.jugador.f--; break;
				case noreste: c_state_N4.jugador.f--; c_state_N4.jugador.c++; break;
				case este: c_state_N4.jugador.c++; break;
				case sureste: c_state_N4.jugador.f++; c_state_N4.jugador.c++; break;
				case sur: c_state_N4.jugador.f++; break;
				case suroeste: c_state_N4.jugador.f++; c_state_N4.jugador.c--; break;
				case oeste: c_state_N4.jugador.c--; break;
				case noroeste: c_state_N4.jugador.f--; c_state_N4.jugador.c--; break;
			}
			break;

		case actTURN_SL:
			a=c_state_N4.jugador.brujula;
			a=(a+7)%8;
			c_state_N4.jugador.brujula=static_cast<Orientacion>(a);
			break;

		case actTURN_SR:
			a=c_state_N4.jugador.brujula;
			a=(a+1)%8;
			c_state_N4.jugador.brujula=static_cast<Orientacion>(a);
			break;

		case actTURN_L:
			a=c_state_N4.jugador.brujula;
			a=(a+6)%8;
			c_state_N4.jugador.brujula=static_cast<Orientacion>(a);
			break;

		case actTURN_R:
			a=c_state_N4.jugador.brujula;
			a=(a+2)%8;
			c_state_N4.jugador.brujula=static_cast<Orientacion>(a);
			break;

		case actSON_FORWARD:
			switch (c_state_N4.sonambulo.brujula){
				case norte: c_state_N4.sonambulo.f--; break;
				case noreste: c_state_N4.sonambulo.f--; c_state_N4.sonambulo.c++; break;
				case este: c_state_N4.sonambulo.c++; break;
				case sureste: c_state_N4.sonambulo.f++; c_state_N4.sonambulo.c++; break;
				case sur: c_state_N4.sonambulo.f++; break;
				case suroeste: c_state_N4.sonambulo.f++; c_state_N4.sonambulo.c--; break;
				case oeste: c_state_N4.sonambulo.c--; break;
				case noroeste: c_state_N4.sonambulo.f--; c_state_N4.sonambulo.c--; break;
			}
			break;

		case actSON_TURN_SL:
			a=c_state_N4.sonambulo.brujula;
			a=(a+7)%8;
			c_state_N4.sonambulo.brujula=static_cast<Orientacion>(a);
			break;

		case actSON_TURN_SR:
			a=c_state_N4.sonambulo.brujula;
			a=(a+1)%8;
			c_state_N4.sonambulo.brujula=static_cast<Orientacion>(a);
			break;

	}
}

void ComportamientoJugador::rellenarMapa(Sensores sensores, vector< vector<unsigned char> > &matriz){
	// Aquí hay que rellenar el mapa con el terreno que hay delante
	// de nuestro personaje.

	switch(c_state_N4.jugador.brujula){
		case norte:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c-1] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c+1] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c-2] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c-1] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c+1] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c+2] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c-3] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c-2] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c-1] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c+1] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c+2] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c+3] = sensores.terreno[15];
			break;

		case noreste:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c+1] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+1] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c+1] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c+2] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c+2] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+2] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c+1] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c+2] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c+3] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c+3] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c+3] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+3] = sensores.terreno[15];
			break;

		case este:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c+1] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+1] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c+1] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c+2] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c+2] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+2] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c+2] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c+2] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c+3] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c+3] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c+3] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+3] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c+3] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c+3] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c+3] = sensores.terreno[15];
			break;

		case sureste:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+1] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c+1] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+2] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c+2] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c+2] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c+1] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c+3] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c+3] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c+3] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c+3] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c+2] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c+1] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c] = sensores.terreno[15];
			break;

		case sur:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c+1] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c-1] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c+2] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c+1] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c-1] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c-2] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c+3] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c+2] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c+1] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c-1] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c-2] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c-3] = sensores.terreno[15];
			break;

		case suroeste:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c-1] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-1] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c-1] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c-2] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c-2] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-2] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c-1] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c-2] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c-3] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c-3] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c-3] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-3] = sensores.terreno[15];
			break;

		case oeste:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c-1] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-1] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c-1] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c-2] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c-2] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-2] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c-2] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c-2] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f+3][c_state_N4.jugador.c-3] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f+2][c_state_N4.jugador.c-3] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f+1][c_state_N4.jugador.c-3] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-3] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c-3] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c-3] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c-3] = sensores.terreno[15];
			break;

		case noroeste:
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c] = sensores.terreno[0];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-1] = sensores.terreno[1];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c-1] = sensores.terreno[2];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c] = sensores.terreno[3];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-2] = sensores.terreno[4];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c-2] = sensores.terreno[5];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c-2] = sensores.terreno[6];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c-1] = sensores.terreno[7];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c] = sensores.terreno[8];
			matriz[c_state_N4.jugador.f][c_state_N4.jugador.c-3] = sensores.terreno[9];
			matriz[c_state_N4.jugador.f-1][c_state_N4.jugador.c-3] = sensores.terreno[10];
			matriz[c_state_N4.jugador.f-2][c_state_N4.jugador.c-3] = sensores.terreno[11];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c-3] = sensores.terreno[12];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c-2] = sensores.terreno[13];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c-1] = sensores.terreno[14];
			matriz[c_state_N4.jugador.f-3][c_state_N4.jugador.c] = sensores.terreno[15];
			break;
	}
}

bool CasillaTransitableN4(const ubicacion &x, const vector<vector<unsigned char> > &mapa){
	return ((mapa[x.f][x.c] != 'P' && mapa[x.f][x.c] != 'M'));
}

stateN4 apply(const Action &a, const stateN4 &st, const vector<vector<unsigned char> > &mapa){
	stateN4 st_result = st;
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
		case actSON_FORWARD:
			sig_ubicacion = NextCasilla(st.sonambulo);
			if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == st.jugador.f && sig_ubicacion.c == st.jugador.c)){
				st_result.sonambulo = sig_ubicacion;
			}
			break;
		case actSON_TURN_SL:
			st_result.sonambulo.brujula = static_cast<Orientacion>((st.sonambulo.brujula + 7) % 8);
			break;
		case actSON_TURN_SR:
			st_result.sonambulo.brujula = static_cast<Orientacion>((st.sonambulo.brujula + 1) % 8);
			break;
	}
	return st_result;
}

list<Action> AnchuraSoloJugadorN4(const stateN4 &inicio, const ubicacion &final, const vector<vector<unsigned char> > &mapa){
	nodeN4 current_node;
	list<nodeN4> frontier;
	set<nodeN4> explored;
	list<Action> plan;
	current_node.st = inicio;
	bool SolutionFound = (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() && !SolutionFound){
		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actFORWARD
		nodeN4 child_forward = current_node;
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
			nodeN4 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end()){
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN4 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end()){
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty()){
			current_node = frontier.front();
			while (!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}

	if (SolutionFound){
		plan = current_node.secuencia;
	}

	return plan;
}

bool ComportamientoJugador::loboCerca(Sensores sensores){
	for (int i=1; i<sensores.superficie.size()-7; ++i)
		if (sensores.superficie[i]=='l')
			return true;

	return false;
}

bool ComportamientoJugador::puedoAvanzar(int num, Sensores sensores, stateN4 &st){
	return ((sensores.terreno[num]== 'T' || sensores.terreno[num]== 'S' || sensores.terreno[num]== 'G' ||
			sensores.terreno[num]== 'D' || sensores.terreno[num]== 'X' || sensores.terreno[num]== 'K'||
			(sensores.terreno[num]== 'A' && st.tiene_bikini) || (sensores.terreno[num]== 'B' && st.tiene_zapatillas) )
			&& sensores.superficie[num]== '_');
}

/* ..................................Implementación desde hpp.............................................. */

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

void ComportamientoJugador::VisualizaPlan(const stateN4 &st, const list<Action> &plan){
	AnularMatriz(mapaConPlan);
	stateN4 cst = st;

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

bool ComportamientoJugador::hayPrecipicioDelante(Sensores sensores){
	bool precipicio = true;
	for (int i=1; i<sensores.terreno.size(); ++i)
		if (sensores.terreno[i]!='P')
			precipicio = false;

	return precipicio;
}

bool ComportamientoJugador::hayLimiteIzquierda(Sensores sensores){
	return ((sensores.terreno[1]=='P' || sensores.terreno[1]=='M')
					&& (sensores.terreno[2]=='P' || sensores.terreno[2]=='M'));
}

bool ComportamientoJugador::hayLimiteDerecha(Sensores sensores){
	return ((sensores.terreno[3]=='P' || sensores.terreno[3]=='M')
					&& (sensores.terreno[2]=='P' || sensores.terreno[2]=='M'));
}

bool ComportamientoJugador::hayHuecoIzquierda(Sensores sensores){
	return ((sensores.terreno[5]=='M' && sensores.terreno[11]=='M' &&
					sensores.terreno[1]!='M' && sensores.terreno[4]!='M' && sensores.terreno[9]!='M' &&
					sensores.terreno[1]!='P' && sensores.terreno[4]!='P' && sensores.terreno[9]!='P') ||
					(sensores.terreno[5]=='P' && sensores.terreno[11]=='P' &&
					sensores.terreno[1]!='P' && sensores.terreno[4]!='P' && sensores.terreno[9]!='P' &&
					sensores.terreno[1]!='M' && sensores.terreno[4]!='M' && sensores.terreno[9]!='M'));
}

bool ComportamientoJugador::hayHuecoDerecha(Sensores sensores){
	return ((sensores.terreno[7]=='M' && sensores.terreno[13]=='M' &&
					sensores.terreno[3]!='M' && sensores.terreno[8]!='M' && sensores.terreno[15]!='M' &&
					sensores.terreno[3]!='P' && sensores.terreno[8]!='P' && sensores.terreno[15]!='P') ||
					(sensores.terreno[7]=='P' && sensores.terreno[13]=='P' &&
					sensores.terreno[3]!='P' && sensores.terreno[8]!='P' &&  sensores.terreno[15]!='P' &&
					sensores.terreno[3]!='M' && sensores.terreno[8]!='M' && sensores.terreno[15]!='M'));
}

void ComportamientoJugador::elegirMovimiento(Action &accion, Sensores sensores, stateN4 &st){

	if (sensores.superficie[1]=='l' || sensores.superficie[2]=='l' || sensores.superficie[3]=='l')
		accion = actIDLE;

	else if (loboCerca(sensores) && !girar_derecha)
		accion = actTURN_L;

	else if (loboCerca(sensores))
		accion = actTURN_R;

	else if (sensores.bateria<500 &&
			(sensores.terreno[9]=='X' ||  sensores.terreno[4]=='X' || sensores.terreno[1]=='X'))
		accion = actTURN_L;

	else if (sensores.bateria<500 &&
			(sensores.terreno[15]=='X' ||  sensores.terreno[8]=='X' || sensores.terreno[3]=='X'))
		accion = actTURN_R;

	else if (sensores.bateria<500 &&
			(sensores.terreno[12]=='X' ||  sensores.terreno[6]=='X' || sensores.terreno[2]=='X'))
		accion = actFORWARD;

	else if ((!st.tiene_zapatillas && zapatillasCercaFrente(sensores, st)) ||
						(!st.tiene_bikini && bikiniCercaFrente(sensores, st)))
		accion = actFORWARD;

	else if (!st.tiene_zapatillas && zapatillasCercaIZQ(sensores, st))
		accion = actTURN_L;

	else if (!st.tiene_zapatillas && zapatillasCercaDCHA(sensores, st))
		accion = actTURN_R;

	else if (!st.tiene_bikini && bikiniCercaIZQ(sensores, st))
		accion = actTURN_L;

	else if (!st.tiene_bikini && bikiniCercaDCHA(sensores, st))
		accion = actTURN_R;

	else if (vecesVisitado(2, sensores, st) < vecesVisitado(1, sensores, st) &&
						vecesVisitado(2, sensores, st) < vecesVisitado(3, sensores, st) && puedoAvanzar(2, sensores, st)) {

		if ((sensores.terreno[3]=='M' && sensores.terreno[5]=='M') ||
				(sensores.terreno[3]=='M' && sensores.terreno[1]=='M'))
			accion = actFORWARD;

		else if ((last_action==actTURN_R || last_action==actTURN_L) && puedoAvanzar(2, sensores, st))
			accion = actFORWARD;

		else if (hayHuecoIzquierda(sensores) && puedoAvanzar(1, sensores, st))
			accion = actTURN_L;

		else if (hayHuecoDerecha(sensores) && puedoAvanzar(3, sensores, st))
			accion = actTURN_R;

		else if (hayLimiteIzquierda(sensores))
			accion = actTURN_R;

		else if (hayLimiteDerecha(sensores))
			accion = actTURN_L;

		else
			accion = actFORWARD;

	}

	else if ((hayPrecipicioDelante(sensores) ||
					(sensores.terreno[1]=='M' && sensores.terreno[2]=='M' && sensores.terreno[3]=='M')))
		if (!girar_derecha)
			accion = actTURN_L;
		else
			accion = actTURN_R;

	else if ((last_action==actTURN_R || last_action==actTURN_L) && puedoAvanzar(2, sensores, st))
			accion = actFORWARD;

	else if (hayHuecoIzquierda(sensores) && puedoAvanzar(1, sensores, st))
		accion = actTURN_L;

	else if (hayHuecoDerecha(sensores) && puedoAvanzar(3, sensores, st))
		accion = actTURN_R;

	else if (vecesVisitado(1, sensores, st) <= vecesVisitado(2, sensores, st) &&
						vecesVisitado(4, sensores, st) < vecesVisitado(6,sensores, st) &&
						vecesVisitado(9, sensores, st) < vecesVisitado(12,sensores, st) && puedoAvanzar(1, sensores, st) &&
						puedoAvanzar(4, sensores, st) && puedoAvanzar(9, sensores, st))
		accion = actTURN_L;

	else if (vecesVisitado(3, sensores, st) <= vecesVisitado(2, sensores, st) &&
						vecesVisitado(8, sensores, st) < vecesVisitado(6,sensores, st) &&
						vecesVisitado(15, sensores, st) < vecesVisitado(12,sensores, st) && puedoAvanzar(3, sensores, st) &&
						puedoAvanzar(8, sensores, st) && puedoAvanzar(15, sensores, st))
		accion = actTURN_R;


	else if (puedoAvanzar(2, sensores, st))
		accion = actFORWARD;

	else if (((sensores.terreno[0]=='K' && sensores.terreno[2]=='B' && sensores.terreno[6]=='B') ||
					(sensores.terreno[0]=='D' && sensores.terreno[2]=='A' && sensores.terreno[6]=='A') )
					&& puedoAvanzar(12, sensores, st))
		accion = actFORWARD;

	else if (((sensores.terreno[0]=='K' && sensores.terreno[1]=='B' && sensores.terreno[4]=='B') ||
					(sensores.terreno[0]=='D' && sensores.terreno[1]=='A' && sensores.terreno[4]=='A') )
					&& puedoAvanzar(9, sensores, st))
		accion = actTURN_L;

	else if (((sensores.terreno[0]=='K' && sensores.terreno[3]=='B' && sensores.terreno[8]=='B') ||
					(sensores.terreno[0]=='D' && sensores.terreno[3]=='A' && sensores.terreno[8]=='A') )
					&& puedoAvanzar(15, sensores, st))
		accion = actTURN_R;

	else if (!girar_derecha)
	accion = actTURN_L;

	else
	accion = actTURN_R;


	if (recargando && sensores.terreno[0] == 'X' && sensores.bateria < 3000)
		accion = actIDLE;
	else
		recargando = false;

	if((sensores.terreno[0] == 'B' and !st.tiene_zapatillas) || (sensores.terreno[0]=='A' and !st.tiene_bikini)
		&& sensores.superficie[2] == '_' )
		if ((last_action==actTURN_R || last_action==actTURN_L) && puedoAvanzar(2, sensores, st))
			accion = actFORWARD;
		else if (puedoAvanzar(1, sensores, st))
				accion=actTURN_L;
			else if (puedoAvanzar(3, sensores, st))
				accion=actTURN_R;
			else
				accion=actFORWARD;


	girar_derecha=(rand()%2==0);
}

bool ComportamientoJugador::puedoCruzarDiagonalIZQSinZapatillas(Sensores sensores, stateN4 &st){
	if (st.tiene_zapatillas)
		return true;
	else{
		return (sensores.terreno[4]!='B' && sensores.terreno[1]!='B');
	}
}

bool ComportamientoJugador::puedoCruzarDiagonalDCHASinZapatillas(Sensores sensores, stateN4 &st){
	if (st.tiene_zapatillas)
		return true;
	else{
		return (sensores.terreno[8]!='B' && sensores.terreno[3]!='B');
	}
}

bool ComportamientoJugador::puedoCruzarDiagonalIZQSinBikini(Sensores sensores, stateN4 &st){
	if(st.tiene_bikini)
		return true;
	else{
		return (sensores.terreno[4]!='A' && sensores.terreno[1]!='A');
	}
}

bool ComportamientoJugador::puedoCruzarDiagonalDCHASinBikini(Sensores sensores, stateN4 &st){
	if(st.tiene_bikini)
		return true;
	else{
		return (sensores.terreno[8]!='A' && sensores.terreno[3]!='A');
	}
}

void ComportamientoJugador::rellenarPrecipicios(vector< vector<unsigned char> > &matriz){
	for (int i=0; i<3; ++i)
		for(int j=0;j<matriz.size();++j){

			matriz[i][j]='P';
			matriz[matriz.size()-i-1][j]='P';
			matriz[j][i]='P';
			matriz[j][matriz.size()-i-1]='P';
		}

}

int ComportamientoJugador::vecesVisitado (int num, Sensores sensores, stateN4 &st){
	// Esta funcion devuelve el numero de veces que se ha visitado una casilla dentro del rango de visión

	if (sensores.terreno[num]!='M' && sensores.terreno[num]!='P'){

			if (sensores.terreno[num]=='B' && !st.tiene_zapatillas)
				return INT_MAX;
			else if (sensores.terreno[num]=='A' && !st.tiene_bikini)
				return INT_MAX;

	switch(sensores.sentido){
		case norte:
			switch(num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c-1];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c+1];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c-2];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c-1];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c+1];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c+2];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c-3];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c-2];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c-1];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c+1];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c+2];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c+3];
					break;
			}
			break;

		case noreste:
			switch(num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c+1];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+1];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c+1];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c+2];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c+2];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+2];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c+1];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c+2];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c+3];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c+3];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c+3];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+3];
					break;
			}
			break;

		case este:
			switch(num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c+1];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+1];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c+1];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c+2];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c+2];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+2];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c+2];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c+2];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c+3];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c+3];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c+3];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+3];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c+3];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c+3];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c+3];
					break;
			}
			break;

		case sureste:
			switch (num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+1];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c+1];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+2];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c+2];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c+2];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c+1];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c+3];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c+3];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c+3];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c+3];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c+2];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c+1];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c];
					break;
			}
			break;

		case sur:
			switch (num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c+1];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c-1];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c+2];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c+1];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c-1];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c-2];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c+3];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c+2];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c+1];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c-1];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c-2];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c-3];
					break;
			}
			break;

		case suroeste:
			switch(num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c-1];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-1];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c-1];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c-2];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c-2];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-2];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c-1];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c-2];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c-3];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c-3];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c-3];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-3];
					break;
			}
			break;

		case oeste:
			switch(num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c-1];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-1];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c-1];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c-2];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c-2];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-2];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c-2];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c-2];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f+3][c_state_N4.jugador.c-3];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f+2][c_state_N4.jugador.c-3];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f+1][c_state_N4.jugador.c-3];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-3];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c-3];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c-3];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c-3];
					break;
			}
			break;

		case noroeste:
			switch(num){
				case 0:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c];
					break;
				case 1:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-1];
					break;
				case 2:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c-1];
					break;
				case 3:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c];
					break;
				case 4:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-2];
					break;
				case 5:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c-2];
					break;
				case 6:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c-2];
					break;
				case 7:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c-1];
					break;
				case 8:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c];
					break;
				case 9:
					return mapaVecesVisitado[c_state_N4.jugador.f][c_state_N4.jugador.c-3];
					break;
				case 10:
					return mapaVecesVisitado[c_state_N4.jugador.f-1][c_state_N4.jugador.c-3];
					break;
				case 11:
					return mapaVecesVisitado[c_state_N4.jugador.f-2][c_state_N4.jugador.c-3];
					break;
				case 12:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c-3];
					break;
				case 13:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c-2];
					break;
				case 14:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c-1];
					break;
				case 15:
					return mapaVecesVisitado[c_state_N4.jugador.f-3][c_state_N4.jugador.c];
					break;
			}
			break;
	}
	}
	return INT_MAX;
}

bool ComportamientoJugador::bikiniCercaIZQ(Sensores sensores, stateN4 &st){
	return ( (sensores.terreno[9]=='K' && sensores.terreno[4]!='M' && sensores.terreno[1]!='M')||
				(sensores.terreno[4]=='K' && sensores.terreno[1]!='M') ||
				sensores.terreno[1]=='K' && puedoAvanzar(1, sensores, st) &&
				puedoAvanzar(4, sensores, st)  && puedoAvanzar(9, sensores, st));
}

bool ComportamientoJugador::bikiniCercaDCHA(Sensores sensores, stateN4 &st){
	return ( (sensores.terreno[15]=='K' && sensores.terreno[8]!='M' && sensores.terreno[3]!='M')||
				(sensores.terreno[8]=='K' && sensores.terreno[3]!='M') ||
				sensores.terreno[3]=='K' && puedoAvanzar(3, sensores, st) &&
				puedoAvanzar(8, sensores, st)  && puedoAvanzar(15, sensores, st));
}

bool ComportamientoJugador::bikiniCercaFrente(Sensores sensores, stateN4 &st){
	return ( (sensores.terreno[12]=='K' && sensores.terreno[6]!='M' && sensores.terreno[2]!='M')||
				(sensores.terreno[6]=='K' && sensores.terreno[2]!='M') ||
				sensores.terreno[2]=='K' && puedoAvanzar(2, sensores, st) &&
				puedoAvanzar(6, sensores, st)  && puedoAvanzar(12, sensores, st));
}

bool ComportamientoJugador::zapatillasCercaIZQ(Sensores sensores, stateN4 &st){
	return ( (sensores.terreno[9]=='D' && sensores.terreno[4]!='M' && sensores.terreno[1]!='M')||
					(sensores.terreno[4]=='D' && sensores.terreno[1]!='M') ||
					sensores.terreno[1]=='D' && puedoAvanzar(1, sensores, st) &&
					puedoAvanzar(4, sensores, st)  && puedoAvanzar(9, sensores, st));
}

bool ComportamientoJugador::zapatillasCercaDCHA(Sensores sensores, stateN4 &st){
	return ( (sensores.terreno[15]=='D' && sensores.terreno[8]!='M' && sensores.terreno[3]!='M')||
					(sensores.terreno[8]=='D' && sensores.terreno[3]!='M') ||
					sensores.terreno[3]=='D' && puedoAvanzar(3, sensores, st) &&
					puedoAvanzar(8, sensores, st)  && puedoAvanzar(15, sensores, st));
}

bool ComportamientoJugador::zapatillasCercaFrente(Sensores sensores, stateN4 &st){
	return ( (sensores.terreno[12]=='D' && sensores.terreno[6]!='M' && sensores.terreno[2]!='M')||
					(sensores.terreno[6]=='D' && sensores.terreno[2]!='M') ||
					sensores.terreno[2]=='D' && puedoAvanzar(2, sensores, st) &&
					puedoAvanzar(6, sensores, st)  && puedoAvanzar(12, sensores, st));
}


