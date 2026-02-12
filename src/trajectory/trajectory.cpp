#include <wolf_controller_utils/trajectory/trajectory.h>

#include <algorithm>
#include <cmath>

using namespace wolf_controller_utils;
using namespace trajectory;

namespace
{
inline void zeroIfRequested(Eigen::Vector6d* out)
{
    if(out)
    {
        out->setZero();
    }
}
} // namespace

void fifthOrderPlanning(double x0, double dx0, double ddx0,
                        double goal, double start_time, double end_time,
                        double time, double& x, double& dx, double& ddx)
{
    Eigen::Matrix6d A;
    A << 1.0000,        0,        0,         0,         0,         0,
            0,    1.0000,         0,         0,         0,         0,
            0,         0,    0.5000,         0,         0,         0,
            -10.0000,   -6.0000,   -1.5000,   10.0000,   -4.0000,    0.5000,
            15.0000,    8.0000,    1.5000,  -15.0000,    7.0000,   -1.0000,
            -6.0000,   -3.0000,   -0.5000,    6.0000,   -3.0000,    0.5000;

    double alpha = (end_time-start_time);
    alpha = std::max(1e-6, alpha);
    double tau = (time - start_time)/alpha; // d/dt = d/d(alpha*tau)
    tau = std::max(0.0, tau);
    tau = std::min(tau, 1.0);

    Eigen::Vector6d b;
    b << x0, dx0*alpha, ddx0*std::pow(alpha,2.0), goal, 0.0, 0.0;

    Eigen::Vector6d coeffs = A*b;

    Eigen::Vector6d t_v, dt_v, ddt_v;
    for(int i = 0; i < 6; i++)
    {
        t_v(i) = std::pow(tau, i);
        dt_v(i) = i > 0 ? std::pow(tau, i-1)*i : 0;
        ddt_v(i) = i > 1 ? std::pow(tau, i-2)*i*(i-1) : 0;

    }

    x = coeffs.dot(t_v);
    dx = coeffs.dot(dt_v)/alpha;
    ddx = coeffs.dot(ddt_v)/(alpha*alpha);
}


Trajectory::WayPoint::WayPoint()
{
    frame_.setIdentity();
    vel_.setZero();
    acc_.setZero();
    time_ = 0.0;
}

Trajectory::Trajectory()
{
    frames_.reserve(10);
}

Trajectory::WayPoint::WayPoint(Eigen::Affine3d frame, double time):
    WayPoint()
{
    this->frame_ = frame;
    this->time_ = time;
}


void Trajectory::addWayPoint(const Trajectory::WayPoint& waypoint, double time_offset)
{
    frames_.push_back(waypoint);
    frames_.back().time_ += time_offset;
    
    sortFrames();
}

void Trajectory::addWayPoint(double time, const Eigen::Affine3d& frame)
{
    WayPoint wp;
    wp.frame_ = frame;
    wp.time_ = time;
    
    addWayPoint(wp);
}

void Trajectory::clear()
{
    frames_.clear();
}

const std::vector< Trajectory::WayPoint >& Trajectory::getWayPoints() const
{
    return frames_;
}

bool Trajectory::isTrajectoryEnded(double time) const
{
    if(frames_.empty())
    {
        return true;
    }

    return time > frames_.back().time_;
}

void Trajectory::sortFrames()
{
    std::sort(frames_.begin(), frames_.end(), [](const WayPoint& w1, const WayPoint& w2){ return w1.time_ < w2.time_; });
}

Eigen::Affine3d Trajectory::evaluate(double time, Eigen::Vector6d * const vel, Eigen::Vector6d * const acc)
{
    if(frames_.empty())
    {
        zeroIfRequested(vel);
        zeroIfRequested(acc);
        return Eigen::Affine3d::Identity();
    }

    /* Find relevant segment (first frame after time) */
    auto it = std::find_if(frames_.begin(),
                           frames_.end(),
                           [&time](const WayPoint& wp){ return wp.time_ > time; });
    
    if(it == frames_.begin()) // trajectory yet to start
    {
        zeroIfRequested(vel);
        zeroIfRequested(acc);
        return it->frame_;
    }
    
    if(it == frames_.end()) // no frames after time (traj is finished)
    {
        zeroIfRequested(vel);
        zeroIfRequested(acc);
        return frames_.back().frame_;
    }
    
    Eigen::Affine3d end = it->frame_;
    Eigen::Affine3d start = (it-1)->frame_;
    double t_end = it->time_;
    double t_start = (it-1)->time_;

    Eigen::Quaterniond q_start(start.linear());
    Eigen::Quaterniond q_end(end.linear());
    
    double tau, dtau, ddtau;
    fifthOrderPlanning(0, 0, 0, 1, t_start, t_end, time, tau, dtau, ddtau);
    
    Eigen::Affine3d interpolated;
    interpolated.setIdentity();
    interpolated.linear() = q_start.slerp(tau, q_end).toRotationMatrix();
    interpolated.translation() = (1 - tau)*start.translation() + tau*end.translation();
    
    if(vel)
    {
        vel->setZero();
        vel->head<3>() = (- dtau)*start.translation() + dtau*end.translation();
    }

    if(acc)
    {
        acc->setZero();
        acc->head<3>() = (- ddtau)*start.translation() + ddtau*end.translation();
    }
    
    return interpolated;
}

int Trajectory::getCurrentSegmentId(double time) const
{
    if(frames_.empty())
    {
        return -1;
    }

    /* Find relevant segment (first frame after time) */
    auto it = std::find_if(frames_.begin(),
                           frames_.end(),
                           [&time](const WayPoint& wp){ return wp.time_ > time; });

    int id = std::distance(frames_.begin(), it) - 1;

    return id;
}


void Trajectory::compute()
{

}
