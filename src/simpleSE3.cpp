#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;


class KinematicCarModel
{
public:

    KinematicCarModel(const ob::StateSpace *space) : space_(space), carLength_(0.2)
    {
    }

    void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
    {
        const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

        dstate.resize(3);
        dstate[0] = u[0] * cos(theta);
        dstate[1] = u[0] * sin(theta);
        dstate[2] = u[0] * tan(u[1]) / carLength_;
    }

    void update(ob::State *state, const std::valarray<double> &dstate) const
    {
        ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
        s.setX(s.getX() + dstate[0]);
        s.setY(s.getY() + dstate[1]);
        s.setYaw(s.getYaw() + dstate[2]);
        space_->enforceBounds(state);
    }

private:

    const ob::StateSpace *space_;
    const double          carLength_;

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

    const ob::StateSpace *space_;
    double                   timeStep_;
    F                        ode_;
};


bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);



    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
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

    EulerIntegrator<KinematicCarModel> integrator_;
};


void planWithSimpleSetup()
{
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([si](const ob::State *state) { return isStateValid(si, state); });

    auto propagator(std::make_shared<DemoStatePropagator>(si));
    ss.setStatePropagator(propagator);

    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.0);
    goal->setY(0.5);
    goal->setYaw(0.0);
    double sst_selection_radius = 0.2;
    double sst_pruning_radius = 0.1;
    auto sst_planner(std::make_shared<oc::SST>(ss.getSpaceInformation()));
    sst_planner->setSelectionRadius(sst_selection_radius); // default 0.2
    sst_planner->setPruningRadius(sst_pruning_radius); // default 0.1
    ss.setPlanner(sst_planner);

    ss.setPlanner(sst_planner);
    ss.setStartAndGoalStates(start, goal, 0.05);

    ss.setup();
    propagator->setIntegrationTimeStep(si->getPropagationStepSize());

    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

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
