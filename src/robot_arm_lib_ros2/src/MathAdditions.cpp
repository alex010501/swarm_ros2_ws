#include <robot_arm_lib_ros2/MathAdditions.h>
#include <iostream>

/**
 * Converts degrees to radians.
 *
 * @param p_deg the value in degrees to be converted
 *
 * @return the value in radians
 */
double MathAdditions::DegToRad(double p_deg)
{
    return p_deg * M_PI / 180.0;
}

/**
 * Converts radians to degrees.
 *
 * @param p_rad the value in radians to be converted
 *
 * @return the value in degrees
 */
double MathAdditions::RadToDeg(double p_rad)
{
    return p_rad * 180.0 / M_PI;
}

/**
 * Calculates the projection of vector `a` onto vector `b`.
 *
 * @param p_a The first vector.
 * @param p_b The second vector.
 *
 * @return The projection of vector `a` onto vector `b`.
 * 
 * @throws std::invalid_argument if the second vector is null.
 */
double MathAdditions::projVector(Eigen::Vector3d p_a, Eigen::Vector3d p_b)
{
    if (p_b.norm() == 0)
    {
        throw std::invalid_argument("Vector b must not be null");
    }
    return p_a.dot(p_b) / p_b.norm();
}

/**
 * Calculates the angle between two vectors around a given axis.
 *
 * @param p_a The first vector.
 * @param p_b The second vector.
 * @param p_Axis The axis around which the angle is calculated.
 *
 * @return The angle between the vectors in radians with direction of rotation.
 *
 * @throws std::invalid_argument if either of the vectors is null or one of the vectors is parallel to the axis.
 */
double MathAdditions::getAngleAroundAxis(Eigen::Vector3d p_a, Eigen::Vector3d p_b, Eigen::Vector3d p_Axis)
{
    if (p_Axis.norm() == 0)
        throw std::invalid_argument("Axis must not be null");
    
    p_Axis.normalize();
    p_a = p_a - MathAdditions::projVector(p_a, p_Axis) * p_Axis;
    p_b = p_b - MathAdditions::projVector(p_b, p_Axis) * p_Axis;
    Eigen::Vector3d lv_c = p_a.cross(p_b);
    if (p_a.norm() == 0 || p_b.norm() == 0)
        throw std::invalid_argument("Vectors must not be parallel to axis");
    else if (p_a.norm()*p_b.norm() == p_a.dot(p_b))
        return 0;
    else if (p_a.norm()*p_b.norm() == -p_a.dot(p_b))
        return M_PI;
    else if (MathAdditions::projVector(p_a, p_b) == 0)
        if (MathAdditions::projVector(lv_c, p_Axis) > 0)
            return M_PI/2;
        else
            return -M_PI/2;
    else
        if (MathAdditions::projVector(lv_c, p_Axis) > 0)
            return acos(p_a.dot(p_b) / (p_a.norm() * p_b.norm()));
        else
            return -acos(p_a.dot(p_b) / (p_a.norm() * p_b.norm()));
}

/**
 * Calculates the angle between two vectors around a given axis.
 *
 * @param p_a The first vector.
 * @param p_b The second vector.
 *
 * @return The angle between the vectors in radians.
 *
 * @throws std::invalid_argument if either of the vectors is null.
 */
double MathAdditions::getAngle(Eigen::Vector3d p_a, Eigen::Vector3d p_b)
{
    if (p_a.norm() == 0 || p_b.norm() == 0)
        throw std::invalid_argument("Vectors must not be null"); 
    else if (p_a.norm()*p_b.norm() == p_a.dot(p_b))
        return 0;
    else if (p_a.norm()*p_b.norm() == -p_a.dot(p_b))
        return M_PI;
    else if (MathAdditions::projVector(p_a, p_b) == 0)
        return M_PI/2;
    else
        return acos(p_a.dot(p_b) / (p_a.norm() * p_b.norm()));
}

/**
 * Generates a 3x3 rotation matrix around the x-axis
 *
 * @param p_angle The angle of rotation in radians
 *
 * @return The rotation matrix
 */
Eigen::Matrix3d MathAdditions::Rx(double p_angle)
{
    Eigen::Matrix3d lv_Rx;
    lv_Rx << 1,            0,             0,
             0, cos(p_angle), -sin(p_angle),
             0, sin(p_angle),  cos(p_angle);
    return lv_Rx;
}

/**
 * Generates a 3x3 rotation matrix around the y-axis
 *
 * @param p_angle The angle of rotation in radians
 *
 * @return The rotation matrix.
 */
Eigen::Matrix3d MathAdditions::Ry(double p_angle)
{
    Eigen::Matrix3d lv_Ry;
    lv_Ry << cos(p_angle), 0, sin(p_angle),
                        0, 1,            0,
            -sin(p_angle), 0, cos(p_angle);
    return lv_Ry;
}

/**
 * Generates a 3x3 rotation matrix around the z-axis
 *
 * @param p_angle The angle of rotation in radians
 *
 * @return The rotation matrix
 */
Eigen::Matrix3d MathAdditions::Rz(double p_angle)
{
    Eigen::Matrix3d lv_Rz;
    lv_Rz << cos(p_angle), -sin(p_angle), 0,
             sin(p_angle),  cos(p_angle), 0,
                        0,             0, 1;
    return lv_Rz;
}

/**
 * Calculates the rotation matrix R(x, y, z) based on the given Euler angles.
 *
 * @param p_x The angle in radians around the x-axis.
 * @param p_y The angle in radians around the y-axis.
 * @param p_z The angle in radians around the z-axis.
 *
 * @return The 3x3 rotation matrix R(x, y, z)
 */
Eigen::Matrix3d MathAdditions::R(double p_x, double p_y, double p_z)
{
    return Rx(p_x) * Ry(p_y) * Rz(p_z);
}

std::vector<double> MathAdditions::make_vector(double p_begin, double p_end, double p_step)
{
    std::vector<double> lv_vector;
    lv_vector.reserve((p_end - p_begin) / p_step + 1);
    while (p_begin <= p_end)
    {
        lv_vector.push_back(p_begin);
        p_begin += p_step;
    }
    return lv_vector;
}

/*std::vector<double> MathAdditions::zero_vector(int p_size)
{
    std::vector<double> lv_vector;
    lv_vector.reserve(p_size);
    for (int i = 0; i < p_size; i++)
    {
        lv_vector.push_back(0.0);
    }
    return lv_vector;
}*/

void MathAdditions::Integrator::init(double p_dt, double p_initValue)
{
    this->dt = p_dt;
    this->IntegratorValue = p_initValue;    
    this->PrevValue = 0;
}

double MathAdditions::Integrator::calculate(double p_funcValue)
{
    this->IntegratorValue += (p_funcValue + this->PrevValue) / 2 * this->dt;
    this->PrevValue = p_funcValue;
    return this->IntegratorValue;
}

void MathAdditions::Derivator::init(double p_dt)
{
    this->dt = p_dt;
    this->PrevValue = 0;
}

double MathAdditions::Derivator::calculate(double p_funcValue)
{
    double derivative = (p_funcValue - this->PrevValue) / this->dt;
    this->PrevValue = p_funcValue;
    return derivative;
}

void MathAdditions::PWM::init(double p_dt, int p_periodMultiplier)
{
    this->dt = p_dt;
    this->period = p_periodMultiplier * p_dt;
    this->elapsedTime = 0;
}

double MathAdditions::PWM::calculate(double p_funcValue)
{
    double signal;

    if (this->elapsedTime < this->period * p_funcValue)
        signal = 1;
    else
        signal = 0;

    this->elapsedTime += this->dt;
    if (this->elapsedTime > this->period)
    {
        this->elapsedTime = 0;
    }
    return signal;
}

MathAdditions::PID_regulator::PID_regulator(double p_Kp, double p_Ki, double p_Kd)
{
    this->setPID(p_Kp, p_Ki, p_Kd);
}

void MathAdditions::PID_regulator::setPID(double p_Kp, double p_Ki, double p_Kd)
{
    this->Kp = p_Kp;
    this->Ki = p_Ki;
    this->Kd = p_Kd;
}

void MathAdditions::PID_regulator::init(double p_dt)
{
    this->integrator.init(p_dt);
    this->derivator.init(p_dt);
}

double MathAdditions::PID_regulator::calculate(double p_signal, double p_dt)
{
    double derivative = this->derivator.calculate(p_signal);
    double integral = this->integrator.calculate(p_signal);
    return this->Kp * p_signal + this->Ki * integral + this->Kd * derivative;
}

MathAdditions::randSignal::randSignal()
{
    // Initialize random coefficients
    this->a = this->getRand(-1.0, 1.0);
    this->b = this->getRand(-0.5, 0.5);
    this->c = this->getRand(-0.1, 0.1);
    // Initialize random frequencies
    int k1 = this->getRand(1, 5);
    int k2 = this->getRand(5, 10);
    int k3 = this->getRand(10, 20);
    this->f1 = k1 * M_PI / 2;
    this->f2 = k2 * M_PI / 2;
    this->f3 = k3 * M_PI / 2;
}

template <typename T>
T MathAdditions::randSignal::getRand(T lower_bound, T upper_bound)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    if constexpr (std::is_integral_v<T>)
    {
        // For integral types (int, long, etc.)
        std::uniform_int_distribution<T> dist(lower_bound, upper_bound);
        return dist(gen);
    }
    else if constexpr (std::is_floating_point_v<T>)
    {
        // For floating-point types (float, double, etc.)
        std::uniform_real_distribution<T> dist(lower_bound, upper_bound);
        return dist(gen);
    }
    else
    {
        // Unsupported type
        throw std::invalid_argument("Unsupported type for getRand");
    }
}

double MathAdditions::randSignal::get_signal(double t)
{
    double firstHarmonic = this->a * sin(this->f1 * t);
    double secondHarmonic = this->b * sin(this->f2 * t);
    double thirdHarmonic = this->c * sin(this->f3 * t);
    // std::cout << "Time " << t << std::endl;
    // std::cout << firstHarmonic << " " << secondHarmonic << " " << thirdHarmonic << std::endl;
    return firstHarmonic + secondHarmonic + thirdHarmonic;
}