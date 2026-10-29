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
actions when available, a results summary, and the complete action timeline.
Override the topic with the experiment_info_topic parameter. View it with:
ros2 topic echo /experiment/info

The exp_runner energy model is configurable at startup with the drive_ma,
charge_ma and wait_ma integer parameters (all in mA). Defaults are 3700,
3500 and 1900 respectively. For example:
ros2 run svea_charging exp_runner.py --ros-args -p drive_ma:=4100 -p charge_ma:=3200 -p wait_ma:=1700
These values are shared by the runner, controllers, planner and simulator.

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

Use a fixed outage schedule instead of generated outages (times are seconds
from scenario start):
python run.py --scenario 1 --outages 300:420,900:1020
python run.py --scenario 1 --outages ""

Print exact realized task durations. In a sweep, also add initial SoC, nominal
tasks, charger outages and exact durations as columns in results.csv:
python run.py --scenario 1 --detailed
python run.py --sweep --seeds 20 --soc-range 70 100 --soc-range 70 100 --detailed

The optimizer_time_limit is the CP-SAT budget for EACH replanning call:
python run.py --scenario 1 --controller OPTIMIZER --planner-time-limit 0.5

The optimizer enforces a 1% minimum SoC and penalizes each percentage point
below 11%. Tune that objective weight with --soc-penalty-weight (default 1000)
or the ROS parameter soc_penalty_weight. The default weight makes one
percentage-point shortfall at one SoC checkpoint equivalent to one second of
makespan in the objective.

The objective also minimizes the sum of the vehicles' completion times, in
addition to the longest completion time. Tune its relative weight with
--total-time-weight (default 1) or the ROS parameter total_time_weight.

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
If a vehicle is already charging, OPTIMIZER and ORACLE may revise its charge
duration at a replan until charge completion is signaled. The planner does not
insert a wait at the charger; it may wait before starting the charger trip.

ORACLE uses exactly the same planner, but supplies the complete future outage
list.

STATIC_OPTIMAL computes one CP-SAT schedule from the scenario's actual initial
SoCs and task lists, with no outages. Only charge
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
An outage prevents a new charge unless the vehicle started driving to the
charger before that outage began. That committed trip may start charging during
the outage, matching the simulator. Otherwise, charging may start at or after
the outage ends. An outage does not interrupt a charge that has already
started. Driving to or from the charger is allowed during an outage.

The planner can wait before a charger trip to avoid an outage. Once the trip
starts, it plans charging immediately on arrival; it does not plan a wait at
the charger. The optimizer enforces at least 5% SoC throughout the sequence and
adds a tunable objective penalty for SoC below 11% at action-boundary
checkpoints. STUPID retains its existing MIN_SOC_MAS and CHARGE_TARGET_MAS
rules.

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
