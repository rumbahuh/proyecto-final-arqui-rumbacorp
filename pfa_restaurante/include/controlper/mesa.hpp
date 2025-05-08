#ifndef CONTROLPER__MESA_HPP_
#define CONTROLPER__MESA_HPP_

#include <iostream>

namespace controlper {

struct Mesa {
  int tamaño;
  bool llena;

  friend std::ostream& operator<<(std::ostream& os, const Mesa& mesa) {
    os << "Mesa(tamaño=" << mesa.tamaño << ", llena=" << (mesa.llena ? "sí" : "no") << ")";
    return os;
  }
};

}  // namespace controlper

#endif  // CONTROLPER__MESA_HPP_

