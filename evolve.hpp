#ifndef evolve_hpp
#define evolve_hpp

#include "functions.hpp"
#include "objects.hpp"

// Funzione evolve che prende in input un vettore (tutti i boids nel piano), un
// singolo boid b_i e restituisce il boid b_i evoluto di un delta t con dei
// parametri s

boid evolve_boid(std::vector<boid> const& flock, boid b_i, double delta_t,
                 stats s) {
  // Preparo le 3 velocità da aggiungere settate a 0
  vel v_sep{0., 0.};
  vel v_all{0., 0.};
  vel v_coe{0., 0.};
  double n = flock.size();
  // L'if serve per verificare che ci sia almeno un altro boid che influenza
  // b_i, altrimenti in v_all e v_coe si dovrebbe dividere per 0. In questo modo
  // se il boid b_i non è influenzato da nessuno rimane uguale
  if (n != 1.) {
    for (boid b_j : flock) {
      // In questo loop prendo sempre lo stesso boid (i-esimo) e da quello
      // calcolo v_sep e v_all per lui (per capire controllare come funzionano
      // le funzioni sep e all)
      v_sep = v_sep + sep(b_i, b_j, s.s, s.d_s);
      v_all = v_all + all(b_i, b_j, n, s.a);
    }
    // v_coe è fuori dal loop più interno perchè è calcolata a partire da un
    // singolo boid e non in relazione agli altri come v_sep e v_all
    v_coe =
        coe(b_i, calc_c_m_b_i(influence(flock, b_i, s.d, s.theta), b_i), s.c);
  }
  b_i.p = {b_i.p.x + (b_i.v * delta_t).v_x, b_i.p.y + (b_i.v * delta_t).v_y};
  b_i.v = b_i.v + v_sep + v_all + v_coe;
  // Effetto pac-man con parametri l_b, r_b, u_b, b_b di stats
  if (b_i.p.x < s.l_b) {
    b_i.p.x = s.r_b - abs(s.l_b - b_i.p.x);
  };
  if (b_i.p.x > s.r_b) {
    b_i.p.x = s.l_b + abs(b_i.p.x - s.r_b);
  };
  if (b_i.p.y > s.u_b) {
    b_i.p.y = s.b_b + abs(b_i.p.y - s.u_b);
  };
  if (b_i.p.y < s.b_b) {
    b_i.p.y = s.u_b - abs(s.b_b - b_i.p.y);
  };
  return b_i;
};

// La funzione evolve_flock prende in input un vettore flock con tutti i boids
// del piano, con un loop evolve ogni singolo boid tenendo conto solo del suo
// range di influenza e restituisce un vettore flock con tutti i boids del piano
// evoluti correttamente tenendo conto solo degli altri boid ad una distanza
// massima d

std::vector<boid> evolve_flock(std::vector<boid> const& flock, double delta_t,
                               stats s) {
  std::vector<boid> f_state = flock;
  int n = flock.size();
  for (int i = 0; i < n; ++i) {
    boid b_i = flock[i];
    f_state[i] =
        evolve_boid(influence(flock, b_i, s.d, s.theta), b_i, delta_t, s);
  };
  return f_state;
};

#endif