#include "PID.h"
#include <numeric>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle)
{
    /**
     * TODO: Initialize PID coefficients (and errors, if needed)
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
        // this->twiddle = { true,           //active
        //                 0.05,             //tolerance
        //                 0.0,              //best_error
        //                 0,                //iteration
        //                 100,              //ignore_initial_iterations
        //                 0,                //index_param
        //                 {0.1, 0.1, 0.1}   //delta_param
        //               };
        this->twiddle.active = true;
        this->twiddle.tolerance = 0.05;
        this->twiddle.best_error = 0.0;
        this->twiddle.iteration = 0;
        this->twiddle.ignore_initial_iterations = 100;
        this->twiddle.index_param = 0;
        this->twiddle.delta_param = {0.1, 0.1, 0.1};
        this->twiddle.state = START;
    }

}

void PID::UpdateError(double cte)
{
    //Update PID errors based on cte.

    this->d_error = cte - this->p_error; // differential error
    this->p_error = cte;                // proportional error
    this->i_error += cte;                // integral error

}

double PID::TotalAverageError(double cte, int n_iter)
{
    if (n_iter >= this->twiddle.ignore_initial_iterations)
    {
        this->total_error += cte * cte;
    }
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
    double sum_threshold = std::accumulate(twiddle.delta_param.begin(), twiddle.delta_param.end(), 0.0);
    if (sum_threshold > twiddle.tolerance)
    {
        switch (twiddle.state)
        {
            case START:
            {
                twiddle.best_error = avrg_error;
                *(this->gains[twiddle.index_param]) += twiddle.delta_param[twiddle.index_param];
                twiddle.state = INCREMENTING;
                break;
            }
            case INCREMENTING:
            {
                if (avrg_error < twiddle.best_error)
                {
                    // incrementing work well, so we increment even more next time
                    twiddle.best_error = avrg_error;
                    twiddle.delta_param[twiddle.index_param] *= 1.1;
                    this->goToNextParam();
                    *(this->gains[twiddle.index_param]) += twiddle.delta_param[twiddle.index_param];
                }
                else
                {
                    // otherwise try in the opposite direction
                    *(this->gains[twiddle.index_param]) -= 2 * twiddle.delta_param[twiddle.index_param];
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
                    twiddle.delta_param[twiddle.index_param] *= 1.1;
                }
                else
                {
                    // otherwise we reset the current parameter to its original value and
                    // reduce the amount modification
                    *(this->gains[twiddle.index_param]) += twiddle.delta_param[twiddle.index_param];
                    twiddle.delta_param[twiddle.index_param] *= 0.9;
                }
                twiddle.state = INCREMENTING;
                this->goToNextParam();
                *(this->gains[twiddle.index_param]) += twiddle.delta_param[twiddle.index_param];
                break;
            }
        }
    }
    else
    {
        twiddle.active = false;
    }

    this->total_error = 0.0;
    std::cout << "\n[Twiddle iteration: " << twiddle.iteration << " ] " << "PID gains :\n"
              << "Kp: " << this->Kp << "\n"
              << "Ki: " << this->Ki << "\n"
              << "Kd: " << this->Kd << "\n"
              << "Best error = " << twiddle.best_error << "\n"
              << "Current error = " << avrg_error << "\n"
              <<std::endl;
}

void PID::goToNextParam()
{
    do
    {
        this->twiddle.index_param = (this->twiddle.index_param + 1) % this->twiddle.delta_param.size();
        // continue until there is a delta_param that we want to change (>0)
    }
    while (this->twiddle.delta_param[this->twiddle.index_param] == 0);

    if (this->twiddle.index_param == 0) {
        ++this->twiddle.iteration;
    }
}