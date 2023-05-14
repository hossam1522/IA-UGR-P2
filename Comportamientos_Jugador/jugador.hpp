#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <cmath>

struct stateN0{
  ubicacion jugador;
  ubicacion sonambulo;

  bool operator== (const stateN0 &x) const{
    if (jugador == x.jugador && sonambulo.f == x.sonambulo.f && sonambulo.c == x.sonambulo.c)
      return true;
    else
      return false;
  }
};

struct nodeN0{
  stateN0 st;
  list<Action> secuencia;

  bool operator== (const nodeN0 &n) const{
    return (st == n.st);
  }

  bool operator< (const nodeN0 &n) const{
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c < n.st.jugador.c)
    return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else
      return false;
  }
};

struct nodeN1{
  stateN0 st;
  list<Action> secuencia;

  bool operator== (const nodeN1 &n) const{
    return (st == n.st);
  }

  bool operator< (const nodeN1 &n) const{
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c < n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f < n.st.sonambulo.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f == n.st.sonambulo.f &&
             st.sonambulo.c < n.st.sonambulo.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f == n.st.sonambulo.f &&
             st.sonambulo.c == n.st.sonambulo.c && st.sonambulo.brujula < n.st.sonambulo.brujula)
      return true;
    else
      return false;

  }
};

struct nodeN2{
  nodeN0 n;
  int coste=0;
  bool tiene_bikini = false, tiene_zapatillas = false;

  bool operator== (const nodeN2 &x) const{
    return (n.st == x.n.st && tiene_bikini == x.tiene_bikini && tiene_zapatillas == x.tiene_zapatillas);
  }

  bool operator< (const nodeN2 &x) const{
    if (n.st.jugador.f < x.n.st.jugador.f)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c < x.n.st.jugador.c)
    return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c && n.st.jugador.brujula < x.n.st.jugador.brujula)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c && n.st.jugador.brujula == x.n.st.jugador.brujula
             && tiene_zapatillas < x.tiene_zapatillas)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c && n.st.jugador.brujula == x.n.st.jugador.brujula
             && tiene_zapatillas == x.tiene_zapatillas && tiene_bikini < x.tiene_bikini)
      return true;
    else
      return false;
  }
};

struct nodeN3{
  nodeN0 n;
  int coste=0;
  int heuristica=0;
  bool tiene_bikini_J = false, tiene_zapatillas_J = false, tiene_bikini_SON = false, tiene_zapatillas_SON = false;

  bool operator== (const nodeN3 &x) const{
    return (n == x.n && tiene_bikini_J == x.tiene_bikini_J && tiene_zapatillas_J == x.tiene_zapatillas_J
            && tiene_bikini_SON == x.tiene_bikini_SON && tiene_zapatillas_SON == x.tiene_zapatillas_SON);
  }

  bool operator< (const nodeN3 &x) const{
    if (n.st.jugador.f < x.n.st.jugador.f)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c < x.n.st.jugador.c)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula < x.n.st.jugador.brujula)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula == x.n.st.jugador.brujula && tiene_zapatillas_J < x.tiene_zapatillas_J)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula == x.n.st.jugador.brujula && tiene_zapatillas_J == x.tiene_zapatillas_J &&
             tiene_bikini_J < x.tiene_bikini_J)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula == x.n.st.jugador.brujula && tiene_zapatillas_J == x.tiene_zapatillas_J &&
             tiene_bikini_J == x.tiene_bikini_J && n.st.sonambulo.f < x.n.st.sonambulo.f)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula == x.n.st.jugador.brujula && tiene_zapatillas_J == x.tiene_zapatillas_J &&
             tiene_bikini_J == x.tiene_bikini_J && n.st.sonambulo.f == x.n.st.sonambulo.f &&
             n.st.sonambulo.c < x.n.st.sonambulo.c)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula == x.n.st.jugador.brujula && tiene_zapatillas_J == x.tiene_zapatillas_J &&
             tiene_bikini_J == x.tiene_bikini_J && n.st.sonambulo.f == x.n.st.sonambulo.f &&
             n.st.sonambulo.c == x.n.st.sonambulo.c && n.st.sonambulo.brujula < x.n.st.sonambulo.brujula)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula == x.n.st.jugador.brujula && tiene_zapatillas_J == x.tiene_zapatillas_J &&
             tiene_bikini_J == x.tiene_bikini_J && n.st.sonambulo.f == x.n.st.sonambulo.f &&
             n.st.sonambulo.c == x.n.st.sonambulo.c && n.st.sonambulo.brujula == x.n.st.sonambulo.brujula &&
             tiene_zapatillas_SON < x.tiene_zapatillas_SON)
      return true;
    else if (n.st.jugador.f == x.n.st.jugador.f && n.st.jugador.c == x.n.st.jugador.c &&
             n.st.jugador.brujula == x.n.st.jugador.brujula && tiene_zapatillas_J == x.tiene_zapatillas_J &&
             tiene_bikini_J == x.tiene_bikini_J && n.st.sonambulo.f == x.n.st.sonambulo.f &&
             n.st.sonambulo.c == x.n.st.sonambulo.c && n.st.sonambulo.brujula == x.n.st.sonambulo.brujula &&
             tiene_zapatillas_SON == x.tiene_zapatillas_SON && tiene_bikini_SON < x.tiene_bikini_SON)
      return true;
    else
      return false;

  }
};

struct stateN4{
  ubicacion jugador;
  ubicacion sonambulo;
  bool tiene_bikini = false, tiene_zapatillas = false;

  bool operator== (const stateN4 &x) const{
    if (jugador == x.jugador && sonambulo.f == x.sonambulo.f && sonambulo.c == x.sonambulo.c)
      return true;
    else
      return false;
  }
};

struct nodeN4{
  stateN4 st;
  list<Action> secuencia;

  bool operator== (const nodeN4 &n) const{
    return (st == n.st);
  }

  bool operator< (const nodeN4 &n) const{
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c < n.st.jugador.c)
    return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else
      return false;
  }
};


/* struct nodeN4{
  stateN4 st;
  list<Action> secuencia;

  bool operator== (const nodeN4 &n) const{
    return (st == n.st);
  }

  bool operator< (const nodeN4 &n) const{
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c < n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f < n.st.sonambulo.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f == n.st.sonambulo.f &&
             st.sonambulo.c < n.st.sonambulo.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c &&
             st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f == n.st.sonambulo.f &&
             st.sonambulo.c == n.st.sonambulo.c && st.sonambulo.brujula < n.st.sonambulo.brujula)
      return true;
    else
      return false;

  }
}; */


class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
      hayPlan = false;
      bien_situado = false;
      ejecutadoWHEREIS = false;


    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const stateN0 &st, const list<Action> &plan);
    void VisualizaPlan(const stateN4 &st, const list<Action> &plan);
    void actualizarVariablesEstado();
    void rellenarMapa(Sensores sensores, vector< vector<unsigned char> > &matriz);
    bool loboCerca(Sensores sensores);
    bool puedoAvanzar(int num, Sensores sensores, stateN4 &st);

  private:
    // Declarar Variables de Estado
    list<Action> plan;
    bool hayPlan, bien_situado, ejecutadoWHEREIS;
    stateN0 c_state;
    stateN4 c_state_N4;
    ubicacion goal;
    Action last_action;

};

#endif
