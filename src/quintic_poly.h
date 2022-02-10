#ifndef QUINTIC_POLY_H
#define QUINTIC_POLY_H

class quintic_poly {
public:
    quintic_poly() = default;
    quintic_poly(double xs, double vxs, double axs, double xe,
                      double vxe, double axe, double t);
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
private:
    double a0, a1, a2, a3, a4, a5;
};

#endif // QUINTIC_POLY_H
