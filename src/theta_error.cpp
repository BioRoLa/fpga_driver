#include <theta_error.hpp>

double theta_error(double start_theta, double goal_theta)
{
    double c1 = cos(start_theta);
    double s1 = sin(start_theta);
    double c2 = cos(goal_theta);
    double s2 = sin(goal_theta);

    double y_frame1 = 0;
    double theta_err = 0;
    y_frame1 = -s1 * c2 + c1 * s2;
    theta_err = asin(y_frame1);
    return theta_err;
}
