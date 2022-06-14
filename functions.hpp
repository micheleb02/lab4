#ifndef functions_hpp
#define funcitions_hpp

#include <cmath>
#include <vector>

#include "objects.hpp"

// Funzione che calcola la distanza fra due boid

double distance(boid b1, boid b2) {
  return sqrt(((b1.p - b2.p).x * (b1.p - b2.p).x) +
              ((b1.p - b2.p).y * (b1.p - b2.p).y));
};

// Funzione che calcola il centro di massa dei boid

pos calc_c_m_b_i(std::vector<boid> const& flock, boid b_i) {
  pos c_m = {0., 0.};
  int n = flock.size();
  for (boid b_j : flock) {
    c_m.x += (b_j.p.x) / (n - 1);
    c_m.y += (b_j.p.y) / (n - 1);
  }
  c_m.x = c_m.x - b_i.p.x / (n - 1);
  c_m.y = c_m.y - b_i.p.y / (n - 1);
  return c_m;
}

// Distanza media

double mean_distance(std::vector<boid> const& v) {
  double n = v.size();
  double dis_tot = 0.;
  for (int i = 0; i < n; i++) {
    int j = i;
    for (; j < n; j++) {
      double dis_tot = dis_tot + distance(v[i], v[j]);
    }
  }
  double m_dis = dis_tot / (n * (n + 1) / 2);
  return m_dis;
}

// Velocità media

vel mean_velocity(std::vector<boid> const& v) {
  double n = v.size();
  vel m_vel{0., 0.};
  for (boid b_i : v) {
    m_vel = m_vel + b_i.v;
  }
  return m_vel * (1 / n);
}

// Velocità di separazione

vel sep(boid b1, boid b2, double s, double d_s) {
  vel v_sep{0., 0.};
  if (distance(b1, b2) < d_s) {
    v_sep = {(b2.p - b1.p).x * s * (-1.), (b2.p - b1.p).y * s * (-1.)};
    return v_sep;
  }
  return v_sep;
}

// Velocità di allineamento

vel all(boid b1, boid b2, double n, double a) {
  vel v_all = (b2.v - b1.v) * (1 / (n - 1)) * a;
  return v_all;
}

// Velocità di coesione

vel coe(boid b1, pos c_m, double c) {
  double v_x = c * (c_m.x - b1.p.x);
  double v_y = c * (c_m.y - b1.p.y);
  vel v_coe = {v_x, v_y};
  return v_coe;
}

// Vision prende in input due boid b_i e b_j e controlla se, dato un angolo
// cieco theta dei boid, b_i riesce a vedere b_j

bool vision(boid b_i, boid b_j, double theta) {
  if (b_i.p.y > b_j.p.y) {
    return abs(b_j.p.x - b_i.p.x) >= ((b_i.p.y - b_j.p.y) * tan(theta / 2));
  } else {
    return true;
  }
}

// La funzione influence prende in input un vettore flock con tutti i boid del
// piano e restituisce un vettore range con solo i boids nel range di influenza
// di un certo boid b_i del piano

std::vector<boid> influence(std::vector<boid> const& flock, boid b_i, double d,
                            double theta) {
  std::vector<boid> range;
  for (boid b_j : flock) {
    if ((distance(b_i, b_j) < d) && vision(b_i, b_j, theta)) {
      range.push_back(b_j);
    }
  };
  return range;
}

#endif