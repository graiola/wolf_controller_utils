/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_CONTROLLER_UTILS_MUX_H
#define WOLF_CONTROLLER_UTILS_MUX_H

// STD
#include <memory>
#include <algorithm>

namespace wolf_controller_utils {

class Input
{
public:

    Input(const double& T = 2.0) {active_=false;t_=0.0;T_=T;}

    ~Input() {}

    virtual void updateInput()=0;

    virtual void resetInput()=0;

    bool isInputActive()
    {
        return active_;
    };

    void increaseTimer(const double& period)
    {
        t_+=period;
    }

    bool isTimerExpired()
    {
        if(t_>=T_)
            return true;
        else
            return false;
    }

    void activate()
    {
        // Reset the timer everytime the input is activated
        t_  = 0.0;
        active_ = true;
    }

    void deactivate()
    {
        active_ = false;
    }


private:
    std::atomic<bool> active_;
    double t_;
    double T_;
};

class Mux
{
public:

    typedef std::list<std::pair<unsigned int,Input*> > list_t;

    Mux() {}

    void addInput(unsigned int priority, Input* input)
    {
        inputs_.push_back(std::make_pair(priority,input));
        inputs_.sort();
    }

    void selectInput(const double& period)
    {
        list_t::iterator it;
        for (it=inputs_.begin(); it!=inputs_.end(); it++)
        {
            it->second->increaseTimer(period);
            if(it->second->isInputActive() || !it->second->isTimerExpired())
            {
                it->second->updateInput();
                it->second->deactivate();
                break;
            }
            else
            {
                it->second->resetInput();
                it->second->updateInput();
                it->second->deactivate();
            }
        }
    }

private:
    list_t inputs_;
};

}; // namespace

#endif
