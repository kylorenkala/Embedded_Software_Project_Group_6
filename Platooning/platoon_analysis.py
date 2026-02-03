import numpy as np
from pycpa import model, analysis

# --------------------------------
# Helper: compute timing statistics
# --------------------------------
def timing_stats(samples_us):
    samples = np.array(samples_us)

    stats = {
        "min": np.min(samples),
        "median": np.median(samples),
        "p95": np.percentile(samples, 95),
        "p99": np.percentile(samples, 99),
        "max": np.max(samples)
    }
    return stats


# --------------------------------
# Load measurement data (µs)
# --------------------------------
logic_samples_us = np.loadtxt("logic_times_us.txt")
comms_samples_us = np.loadtxt("comms_times_us.txt")

logic_stats = timing_stats(logic_samples_us)
comms_stats = timing_stats(comms_samples_us)

# --------------------------------
# Print statistics (as requested)
# --------------------------------
print("Logic task timing:")
print(f"min   = {logic_stats['min']:.0f} µs")
print(f"median= {logic_stats['median']:.0f} µs")
print(f"P95   = {logic_stats['p95']:.0f} µs")
print(f"P99   = {logic_stats['p99']:.0f} µs")
print(f"max   = {logic_stats['max']:.0f} µs\n")

print("Comms task timing:")
print(f"min   = {comms_stats['min']:.0f} µs")
print(f"median= {comms_stats['median']:.0f} µs")
print(f"P95   = {comms_stats['p95']:.0f} µs")
print(f"P99   = {comms_stats['p99']:.0f} µs")
print(f"max   = {comms_stats['max']:.0f} µs\n")

# --------------------------------
# Use MAX as WCET (engineering WCET)
# --------------------------------
WCET_logic = logic_stats["max"] * 1e-6   # seconds
WCET_comms = comms_stats["max"] * 1e-6   # seconds

PERIOD = 50e-3  # 50 ms

# --------------------------------
# pyCPA system model
# --------------------------------
system = model.System()
cpu = model.Resource("CPU")
system.bind_resource(cpu)

logic = model.Task(
    name="Logic",
    wcet=WCET_logic,
    period=PERIOD
)
logic.bind_resource(cpu)
logic.priority = 1

comms = model.Task(
    name="Comms",
    wcet=WCET_comms,
    period=PERIOD
)
comms.bind_resource(cpu)
comms.priority = 1

system.add_task(logic)
system.add_task(comms)

# --------------------------------
# Response-Time Analysis
# --------------------------------
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
