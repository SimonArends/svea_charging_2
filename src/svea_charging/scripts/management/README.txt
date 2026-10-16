SMART CHARGING ROLLING-HORIZON SIMULATOR
========================================

Files
-----
scenario.py       Scenario generation, vehicle state, outages and action types.
planner.py        Full-horizon CP-SAT planner, reinitialized from the current state.
controllers.py    STUPID, OPTIMIZER, ORACLE and STATIC_OPTIMAL controllers.
simulator.py      Event-driven execution and rolling-horizon replanning.
run.py            Single-scenario and sweep entry point.

Live experiment status
----------------------
The ROS 2 experiment runner publishes a readable live summary as std_msgs/String
on /experiment/info by default. It includes elapsed time, charger outages,
vehicle action and SoC, recent completed-task timing and SoC, and future plan
actions when available. Override the topic with the experiment_info_topic
parameter. View it with:
ros2 topic echo /experiment/info

Runner commands are republished at the experiment update rate. Transport
length is retried briefly when a delivery starts. After each delivery, the
runner waits 2 seconds by default before starting another delivery; adjust
transport_reset_wait_s or transport_length_retry_s to change these intervals.

The charger is considered available to the next vehicle as soon as the runner
publishes charge_done=true; the runner does not wait for parked_after_charge
before planning the next approach. OPTIMIZER and ORACLE show their complete
remaining solver schedule, with action durations and expected SoC at each
action boundary, in the plan section of /experiment/info. Future actions are
for inspection only; the runner still executes only immediate actions.

Install
-------
pip install ortools

Run one scenario
----------------
python run.py --scenario 1
python run.py --scenario 1 --controller STUPID
python run.py --scenario 1 --controller OPTIMIZER
python run.py --scenario 1 --controller ORACLE
python run.py --scenario 1 --controller STATIC_OPTIMAL

Randomize each vehicle's initial SoC independently as an inclusive whole
percentage within its range:
python run.py --scenario 1 --soc-range 70 100 --soc-range 70 100

Print exact realized task durations. In a sweep, also add initial SoC, nominal
tasks, charger outages and exact durations as columns in results.csv:
python run.py --scenario 1 --detailed
python run.py --sweep --seeds 20 --soc-range 70 100 --soc-range 70 100 --detailed

The optimizer_time_limit is the CP-SAT budget for EACH replanning call:
python run.py --scenario 1 --controller OPTIMIZER --planner-time-limit 0.5

Controller semantics
--------------------
STUPID applies a reserve-aware rule independently to each free vehicle. It
delivers the next task when current SoC is sufficient; otherwise it waits for
currently known charger unavailability or drives to charge. It does not
anticipate other vehicles' future charger demand, so simultaneous demand can
cause a queue or an infeasible run. At the charger it charges for remaining
nominal work plus the drive away (or to full if that cannot fit in one battery).

OPTIMIZER uses the same CP-SAT model on every rolling-horizon decision. It does
NOT know future outages. The planner is initialized from the current SoC,
location, remaining task index and remaining task durations.

ORACLE uses exactly the same planner, but supplies the complete future outage
list.

STATIC_OPTIMAL computes one CP-SAT schedule before scenario generation, using
the default nominal task list, 75% initial SoC, and no outages. Only charge
windows are reserved: early arrivals wait for their slot, late arrivals charge
only until its end, and vehicles leave when a slot expires. An outage can shift
unstarted slots without changing their durations. If the vehicle has not yet
departed, the shifted slot includes the charger drive after the outage; an
already-started trip does not add that drive a second time. Deliveries and travel follow
the planned order but proceed immediately when the preceding action completes;
they do not wait for nominal timestamps. This controller never re-optimizes, so
realized SoC or task-duration differences can still make the fixed plan infeasible.

Outage rule
-----------
An outage prevents a NEW CHARGE from starting during the outage. It does NOT
interrupt a charge that has already started. The optimizer waits before driving
to the charger when it can predict unavailability; a wait at the charger is
needed only if an outage begins during the drive. The CP-SAT charge constraint is:
    charge_start < outage_start OR charge_start >= outage_end
A charge may legally end after an outage begins.

The simulator applies the same rule when an action actually starts. Driving to
or from the charger is allowed during an outage.

Rolling horizon
---------------
The planner may optimize the complete remaining mission, but the simulator only
commits the first atomic action for each currently free vehicle. Active actions
are frozen. At an action completion, outage transition, or other state change,
the planner is called again using the new current state.

Delivery uncertainty
--------------------
Actual delivery durations are generated once per scenario and reused by all
controllers. The planner uses nominal task durations; the simulator executes
the realized durations. This makes controller comparisons fair.

Metrics
-------
Makespan, total time spent waiting, transport-time fraction, charger utilization,
charging visits and failure reason are reported. Waiting time is summed across
all vehicles from their executed WAIT actions.
