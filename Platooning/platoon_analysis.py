import numpy as np
from pycpa import model, analysis

# ==============================
# CONFIGURATION
# ==============================

INCLUDE_INPUT_TASK = True        # ← toggle this
LOGIC_PERIOD = 50e-3             # 50 ms
COMMS_PERIOD = 50e-3             # 50 ms
INPUT_MIN_INTERARRIVAL = 100e-3  # 100 ms (conservative)

# ==============================
# STATISTICS FUNCTION
# ==============================

def timing_stats(samples_us):
    samples = np.array(samples_us)
    return {
        "min": np.min(samples),
        "median": np.median(samples),
        "p95": np.percentile(samples, 95),
        "p99": np.percentile(samples, 99),
        "max": np.max(samples),
    }

def print_stats(name, stats):
    print(f"{name} timing:")
    print(f"min   = {stats['min']:.0f} µs")
    print(f"median= {stats['median']:.0f} µs")
    print(f"P95   = {stats['p95']:.0f} µs")
    print(f"P99   = {stats['p99']:.0f} µs")
    print(f"max   = {stats['max']:.0f} µs\n")

# ==============================
# LOAD MEASUREMENTS
# ==============================

logic_samples = np.loadtxt("logic_times_us.txt")
comms_samples = np.loadtxt("comms_times_us.txt")

logic_stats = timing_stats(logic_samples)
comms_stats = timing_stats(comms_samples)

print_stats("Logic", logic_stats)
print_stats("Comms", comms_stats)

if INCLUDE_INPUT_TASK:
    input_samples = np.loadtxt("input_times_us.txt")
    input_stats = timing_stats(input_samples)
    print_stats("Input", input_stats)

# ==============================
# WCET (use MAX)
# ==============================

WCET_logic = logic_stats["max"] * 1e-6
WCET_comms = comms_stats["max"] * 1e-6
WCET_input = input_stats["max"] * 1e-6 if INCLUDE_INPUT_TASK else None

# ==============================
# pyCPA MODEL
# ==============================

system = model.System()
cpu = model.Resource("CPU")
system.bind_resource(cpu)

# ---- Logic Task (periodic) ----
logic = model.Task(
    name="Logic",
    wcet=WCET_logic,
    period=LOGIC_PERIOD
)
logic.priority = 2
logic.bind_resource(cpu)
system.add_task(logic)

# ---- Communication Task (periodic) ----
comms = model.Task(
    name="Comms",
    wcet=WCET_comms,
    period=COMMS_PERIOD
)
comms.priority = 2
comms.bind_resource(cpu)
system.add_task(comms)

# ---- Input Task (sporadic, optional) ----
if INCLUDE_INPUT_TASK:
    input_task = model.Task(
        name="Input",
        wcet=WCET_input,
        period=INPUT_MIN_INTERARRIVAL  # sporadic modeled as min inter-arrival
    )
    input_task.priority = 1  # lowest priority
    input_task.bind_resource(cpu)
    system.add_task(input_task)

# ==============================
# RESPONSE-TIME ANALYSIS
# ==============================

rta = analysis.ResponseTimeAnalysis()

print("=== pyCPA Results ===\n")

total_util = 0.0

for task in system.tasks:
    R = rta.compute_response_time(task)
    util = task.wcet / task.period
    total_util += util

    print(f"Task: {task.name}")
    print(f"  WCET        : {task.wcet * 1e6:.0f} µs")
    print(f"  Period      : {task.period * 1e3:.0f} ms")
    print(f"  Deadline    : {task.period * 1e3:.0f} ms (implicit)")
    print(f"  ResponseTime: {R * 1e6:.0f} µs")
    print(f"  Utilization : {util * 100:.3f} %\n")

print(f"Total CPU Load: {total_util * 100:.3f} %")
