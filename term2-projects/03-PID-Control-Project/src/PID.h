#ifndef PID_H
#define PID_H
#include <vector>

enum TwiddleState
{
    START,
    INCREMENTING,
    DECREMENTING
};

struct TwiddleResources
{
    bool active;
    double tolerance;
    double minimum_error;
    double best_error;
    int iteration;
    int ignore_initial_iterations;
    int force_run;
    int index_gains;
    std::vector<double> delta_gains;
    TwiddleState state;
};

class PID {
    public:
        /**
         * Constructor
         */
        PID();

        /**
         * Destructor.
         */
        virtual ~PID();

        /**
         * Initialize PID.
         * @param (Kp_, Ki_, Kd_) The initial PID coefficients
         */
        void Init(double Kp_, double Ki_, double Kd_, bool twiddle);

        /**
         * Update the PID error variables given cross track error.
         * @param cte The current cross track error
         */
        void UpdateError(double cte);

        /**
         * Calculate the total average PID error.
         * @output The total average PID error
         */
        double TotalAverageError(double cte, int n_iter);

        /**
         * @brief Compute the control action
         * @output the steering control
         */
        double ComputeControl();

        /**
         * @brief Twiddle algorithm for PID coefficients tuning
         *
         */
        void Twiddle(double avrg_error);

        /**
         * Twiddle resources parameters
         */
        TwiddleResources twiddle;

    private:
        /**
         * @brief Change index of the next gain to be tuned
         */
        void goToNextGain();

        /**
         * PID Errors
         */
        double p_error;
        double i_error;
        double d_error;

        double total_error;

        /**
         * PID Coefficients
         */
        double Kp;
        double Ki;
        double Kd;
        std::vector<double*> gains = {&Kp, &Ki, &Kd};

        /**
         * Steering actuator saturation
         */
        const double MIN_CONTROL = -1.0;
        const double MAX_CONTROL = 1.0;
};

#endif  // PID_H