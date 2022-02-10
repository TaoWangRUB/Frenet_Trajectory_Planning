#ifndef QUARTIC_POLY_H
#define QUARTIC_POLY_H


class quartic_poly {
public:
    quartic_poly() = default;
    quartic_poly(double xs, double vxs, double axs, double vxe,
                      double axe, double t);
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
private:
    double a0, a1, a2, a3, a4;
};

#endif // QUARTIC_POLY_H
