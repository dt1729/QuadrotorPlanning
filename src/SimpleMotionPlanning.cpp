// https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo/src/hovering_example.cpp

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/PathControl.h>
#include <ompl/config.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;


class QuadRotorModel
{
public:

    QuadRotorModel(const ob::StateSpace *space) : space_(space)
    {
    }

    void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
    {
        const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        auto angles = state->as<ob::SE3StateSpace::StateType>();
        tf::Quaternion q(angles->rotation().x, angles->rotation().y, angles->rotation().z, angles->rotation().w);
        q.normalize();
        tf::Matrix3x3 m(q);
        double roll = 0, pitch = angles->rotation().y, yaw = angles->rotation().z;
        m.getRPY(roll, pitch, yaw);
        /*  access quaternion as angles.x, angles.y, angles.z, angles.w convert these to radians and do integration*/
        dstate.resize(6);
        dstate[0] = u[0] * cos(yaw) * cos(pitch);
        dstate[1] = u[0] * sin(yaw) * cos(pitch);
        dstate[2] = u[0] * sin(pitch);
        dstate[3] = u[1];
        dstate[4] = u[2];
        dstate[5] = 0;
    }

    void update(ob::State *state, const std::valarray<double> &dstate) const
    {
        ob::SE3StateSpace::StateType &s = *state->as<ob::SE3StateSpace::StateType>();
        s.setXYZ(s.getX() + dstate[0], s.getY() + dstate[1], s.getZ() + dstate[2]);
        tf::Quaternion q(s.rotation().x, s.rotation().y, s.rotation().z, s.rotation().w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pitch += dstate[3]; yaw += dstate[4];  roll = 0.000;
        if(pitch > 3.14){
            pitch = -3.14 + (pitch - 3.14);
        }
        else if(pitch < -3.14){
            pitch = 3.14 - (3.14 - pitch);
        }
        if(yaw > 3.14){
            yaw = -3.14 + (yaw - 3.14);
        }
        else if(yaw < -3.14){
            yaw = 3.14 - (3.14 - yaw);
        }
        tf::Quaternion q1; q1.setRPY(roll, pitch, yaw);
        q1.normalize();
        s.rotation().setAxisAngle(q1.x(), q1.y(), q1.z(), q1.w());
        space_->enforceBounds(state);
    }

private:
    const ob::StateSpace *space_;
};


template<typename F>
class EulerIntegrator
{
public:

    EulerIntegrator(const ob::StateSpace *space, double timeStep) : space_(space), timeStep_(timeStep), ode_(space)
    {
    }

    void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
    {
        double t = timeStep_;
        std::valarray<double> dstate;
        space_->copyState(result, start);
        while (t < duration + std::numeric_limits<double>::epsilon())
        {
            ode_(result, control, dstate);
            ode_.update(result, timeStep_ * dstate);
            t += timeStep_;
        }
        if (t + std::numeric_limits<double>::epsilon() > duration)
        {
            ode_(result, control, dstate);
            ode_.update(result, (t - duration) * dstate);
        }
    }

    double getTimeStep() const
    {
        return timeStep_;
    }

    void setTimeStep(double timeStep)
    {
        timeStep_ = timeStep;
    }

private:

    const ob::StateSpace    *space_;
    double                   timeStep_;
    F                        ode_;
};


bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    double x = pos->values[0], y = pos->values[1], z = pos->values[2];
    if(x < 5 && x > 4 && y < 5 && y > 1 && z <= 5 && z > 0){
        return false;
    }
    if(x < 9 && x > 8 && y < 5 && y > 1 && z <= 10 && z > 5){
        return false;
    }
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state)  && (const void*)rot != (const void*)pos;
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 3) // Change this constant
    {
    }
};

class DemoStatePropagator : public oc::StatePropagator
{
public:

    DemoStatePropagator(oc::SpaceInformation *si) : oc::StatePropagator(si),
                                                    integrator_(si->getStateSpace().get(), 0.0)
    {
    }

    void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override
    {
        integrator_.propagate(state, control, duration, result);
    }

    void setIntegrationTimeStep(double timeStep)
    {
        integrator_.setTimeStep(timeStep);
    }

    double getIntegrationTimeStep() const
    {
        return integrator_.getTimeStep();
    }

    EulerIntegrator<QuadRotorModel> integrator_;
};


void planWithSimpleSetup()
{
    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, 0.0);
    bounds.setHigh(0,10.0);
    bounds.setLow(1,0.0);
    bounds.setHigh(1,10.0);
    bounds.setLow(2,0.5);
    bounds.setHigh(2,10.0);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(3);
    cbounds.setLow(0,-1.0);
    cbounds.setHigh(0,1.0); 
    cbounds.setLow(1,-0.3);
    cbounds.setHigh(1,0.3); 
    cbounds.setLow(2,-0.3);
    cbounds.setHigh(2,0.3);


    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([si](const ob::State *state) { return isStateValid(si, state); });

    auto propagator(std::make_shared<DemoStatePropagator>(si));
    ss.setStatePropagator(propagator);

    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setX(1.0);
    start->setY(1.0);
    start->setZ(5.0);
    start->rotation().setAxisAngle(0,0,0,1);

    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setX(10.0);
    goal->setY(3.5);
    goal->setZ(10.0);
    goal->rotation().setAxisAngle(0,0,0,1);

    double sst_selection_radius = 4;
    double sst_pruning_radius = 2;
    double epsilon1 = 0.9, epsilon2 = 0.9; 
    ob::PlannerStatus solved; 

    for(int  i = 0; i < 100; i++){
        auto sst_planner(std::make_shared<oc::SST>(ss.getSpaceInformation()));
        sst_planner->setSelectionRadius(sst_selection_radius); // default 0.2
        sst_planner->setPruningRadius(sst_pruning_radius); // default 0.1

        ss.setPlanner(sst_planner);
        ss.setStartAndGoalStates(start, goal, 1);

        ss.setup();
        propagator->setIntegrationTimeStep(si->getPropagationStepSize());

        solved = ss.solve(0.1);
        std::cout << "Selection Radius: " << sst_selection_radius << " Pruning Radius: " << sst_pruning_radius << std::endl;
        sst_selection_radius *= epsilon1;
        sst_pruning_radius *= epsilon2;
    }

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        oc::PathControl myPath = ss.getSolutionPath();
        auto cc = myPath.getControls()[0]->as<oc::RealVectorControlSpace::ControlType>()->values;
        for(int i = 0; i < 3; i++){
            std::cout << "Controls: " << cc[i] << std::endl;
        }
        std::cout << "Duration: " << myPath.getControlDuration(0) << std::endl;
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    planWithSimpleSetup();

    return 0;
}