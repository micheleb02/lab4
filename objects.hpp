#ifndef objects_hpp
#define objects_hpp

// Da qui a riga 24 definisco la velocità con le 3 operazioni che servono

struct vel {
  double v_x{};
  double v_y{};
};

vel operator+(vel v1, vel v2) {
  vel res = {v1.v_x + v2.v_x, v1.v_y + v2.v_y};
  return res;
}

vel operator-(vel v1, vel v2) {
  vel res = {v1.v_x - v2.v_x, v1.v_y - v2.v_y};
  return res;
}

vel operator*(vel v1, double k) {
  vel res = {k * v1.v_x, k * v1.v_y};
  return res;
}

// Da qui a riga 46 definisco la posizione con le 3 opearazioni che servono

struct pos {
  double x{};
  double y{};
};

pos operator+(pos p1, pos p2) {
  pos res = {p1.x + p2.x, p1.y + p2.y};
  return res;
}

pos operator-(pos p1, pos p2) {
  pos res = {p1.x - p2.x, p1.y - p2.y};
  return res;
}

pos operator*(pos p1, double k) {
  pos res = {k * p1.x, k * p1.y};
  return res;
}

struct boid {
  pos p{};
  vel v{};
};

// Struct che contiene tutti i parametri necessari così non bisogna fare
// funzioni con 10000 input

struct stats {
  double d_s{};  // Parametro v_sep
  double d{};    // Parametro per ogni v
  double s{};
  double a{};
  double c{};
  double l_b{};    // Bordo sinistro
  double r_b{};    // Bordo destro
  double u_b{};    // Bordo superiore
  double b_b{};    // Bordo inferiore
  double theta{};  // Angolo cieco dei boid
};

#endif