#include "PID.h"
#include <numeric>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle)
{
    /**
     * Initialize PID coefficients (and errors, if needed)
     */
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;

    this->total_error = 0.0;

    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;

    if (twiddle)
    {
        this->twiddle.active = true;
        this->twiddle.tolerance = 0.05;
        this->twiddle.minimum_error = 2.5;
        this->twiddle.best_error = 0.0;
        this->twiddle.iteration = 0;
        this->twiddle.ignore_initial_iterations = 100;
        this->twiddle.force_run = 1200;
        this->twiddle.index_gains = 0;
        this->twiddle.delta_gains = std::vector<double>(this->gains.size(), 0.1);
        this->twiddle.state = START;
    }
    else
    {
        this->twiddle.active = false;
    }

}

void PID::UpdateError(double cte)
{
    //Update PID errors based on cte.

    this->d_error = cte - this->p_error;    // differential error
    this->p_error = cte;                    // proportional error
    this->i_error += cte;                   // integral error

}

double PID::TotalAverageError(double cte, int n_iter)
{
    this->total_error += std::abs(cte);
    return this->total_error/n_iter;
}

double PID::ComputeControl()
{
    double control = - (this->Kp * this->p_error) - (this->Kd * this->d_error) - (this->Ki * this->i_error);
    // steering actuator saturation
    if (control < this->MIN_CONTROL)
    {
      control = this->MIN_CONTROL;
    }
    else if (control > this->MAX_CONTROL)
    {
      control = this->MAX_CONTROL;
    }
    return control;
}

void PID::Twiddle(double avrg_error)
{
    double sum_threshold = std::accumulate(twiddle.delta_gains.begin(), twiddle.delta_gains.end(), 0.0);
    if (sum_threshold > twiddle.tolerance)
    {
        switch (twiddle.state)
        {
            case START:
            {
                twiddle.best_error = avrg_error;
                *(this->gains[twiddle.index_gains]) += twiddle.delta_gains[twiddle.index_gains];
                twiddle.state = INCREMENTING;
                break;
            }
            case INCREMENTING:
            {
                if (avrg_error < twiddle.best_error)
                {
                    // incrementing work well, so we increment even more next time
                    twiddle.best_error = avrg_error;
                    twiddle.delta_gains[twiddle.index_gains] *= 1.1;
                    this->goToNextGain();
                    *(this->gains[twiddle.index_gains]) += twiddle.delta_gains[twiddle.index_gains];
                }
                else
                {
                    // otherwise try in the opposite direction
                    *(this->gains[twiddle.index_gains]) -= 2 * twiddle.delta_gains[twiddle.index_gains];
                    twiddle.state = DECREMENTING;
                }
                break;
            }
            case DECREMENTING:
            {
                if (avrg_error < twiddle.best_error)
                {
                    // decrementing work well, so we increment even more next time
                    twiddle.best_error = avrg_error;
                    twiddle.delta_gains[twiddle.index_gains] *= 1.1;
                }
                else
                {
                    // otherwise we reset the current parameter to its original value and
                    // reduce the amount modification
                    *(this->gains[twiddle.index_gains]) += twiddle.delta_gains[twiddle.index_gains];
                    twiddle.delta_gains[twiddle.index_gains] *= 0.9;
                }
                twiddle.state = INCREMENTING;
                this->goToNextGain();
                *(this->gains[twiddle.index_gains]) += twiddle.delta_gains[twiddle.index_gains];
                break;
            }
        }
        std::cout << "\n[Twiddle] iteration: " << twiddle.iteration << "." << twiddle.index_gains << "\n"
                  << "PID gains :" << std::endl;
    }
    else
    {
        twiddle.active = false;
        std::cout << "\n[Twiddle] iteration: " << twiddle.iteration << "\n"
                  << "------BEST SOLUTION------" << "\n"
                  << "PID gains :" << std::endl;
    }

    this->total_error = 0.0;
    twiddle.iteration = 0;
    std::cout << "Kp: " << this->Kp << "\n"
              << "Ki: " << this->Ki << "\n"
              << "Kd: " << this->Kd << "\n"
              << "Best error = " << twiddle.best_error << "\n"
              << "Current error = " << avrg_error << "\n"
              <<std::endl;
}

void PID::goToNextGain()
{
    do
    {
        this->twiddle.index_gains = (this->twiddle.index_gains + 1) % this->twiddle.delta_gains.size();
        // continue until there is a delta_gains that we want to change (>0)
    }
    while (this->twiddle.delta_gains[this->twiddle.index_gains] == 0);
}