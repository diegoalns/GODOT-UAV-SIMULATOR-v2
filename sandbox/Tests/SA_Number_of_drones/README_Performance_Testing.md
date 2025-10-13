# Simulated Annealing Performance Testing Framework

This framework provides comprehensive performance testing for the Simulated Annealing (SA) algorithm used in UAV route planning. It evaluates the computational performance with varying numbers of drones (10-100 in steps of 10) while maintaining a consistent airspace graph structure.

## Key Features

- **Single Graph Generation**: The airspace graph is built once and reused for all tests, ensuring consistent testing conditions
- **Scalable Testing**: Tests drone counts from 10 to 100 in steps of 10
- **Statistical Reliability**: Multiple runs per test configuration for robust statistics
- **Comprehensive Metrics**: Measures execution time, solution quality, conflicts, and convergence
- **Automated Analysis**: Statistical analysis with mean, standard deviation, min/max values
- **Data Export**: Results saved to CSV files for further analysis
- **Visualization**: Performance plots showing scalability trends

## Files Description

### Core Framework Files

1. **`SA_Performance_Test.py`** - Main performance testing framework
   - `UAVGraphBuilder`: Builds the 3D airspace lattice graph
   - `UAVRouteGenerator`: Generates UAV routes for testing
   - `SimulatedAnnealingOptimizer`: SA algorithm implementation
   - `PerformanceTestRunner`: Main test orchestration and analysis

2. **`run_performance_test.py`** - Full performance test runner
   - Tests drone counts: 10, 20, 30, ..., 100
   - 3 runs per test configuration
   - Complete analysis and visualization

3. **`quick_test.py`** - Quick validation test
   - Tests smaller drone counts (5, 10, 15)
   - 2 runs per configuration
   - For framework validation

## Usage Instructions

### Quick Start

1. **Run the quick validation test first**:
   ```bash
   python quick_test.py
   ```
   This validates the framework with smaller tests (takes ~1-2 minutes).

2. **Run the full performance test**:
   ```bash
   python run_performance_test.py
   ```
   This runs the complete evaluation (may take 10-30 minutes depending on system).

### Custom Testing

You can also run custom tests by importing the framework:

```python
from SA_Performance_Test import PerformanceTestRunner

# Initialize test runner
test_runner = PerformanceTestRunner()
test_runner.setup()  # Builds graph once

# Run custom test
drone_counts = [10, 25, 50, 75, 100]  # Custom drone counts
runs_per_test = 5  # More runs for better statistics

test_runner.run_performance_tests(drone_counts, runs_per_test)
test_runner.analyze_results()
test_runner.save_results("custom_results.csv")
test_runner.plot_results()
```

## Graph Structure

The framework uses a 3D lattice graph representing airspace:

- **Dimensions**: 10×10×3 grid (latitude × longitude × altitude)
- **Geographic Bounds**: 
  - Latitude: 40.6042° to 40.6125° (LaGuardia Airport area)
  - Longitude: -73.9458° to -73.9292°
  - Altitude: 0 to 400 meters
- **Node Availability**: 75% of nodes are available (simulating obstacles)
- **Connectivity**: Each node connects to up to 26 neighbors (3D cube)
- **Edge Weights**: 3D geodesic distances

## Performance Metrics

The framework measures several key performance indicators:

### Timing Metrics
- **SA Execution Time**: Time spent in the SA algorithm
- **Route Generation Time**: Time to generate initial routes
- **Total Time**: Complete test execution time

### Solution Quality Metrics
- **Final Cost**: Total solution cost (path length + conflict penalties)
- **Conflicts**: Number of spatial-temporal conflicts
- **Total Path Length**: Sum of all UAV path lengths
- **Convergence**: Number of SA iterations

### Success Metrics
- **Success Rate**: Percentage of successful optimizations
- **Route Generation Success**: Ability to generate requested routes

## Output Files

The framework generates several output files:

1. **Detailed Results CSV**: `sa_performance_results_YYYYMMDD_HHMMSS.csv`
   - Complete test results for each individual run
   - Columns: num_drones, sa_time, final_cost, conflicts, iterations, etc.

2. **Summary Statistics CSV**: `sa_performance_results_YYYYMMDD_HHMMSS_summary.csv`
   - Aggregated statistics by drone count
   - Mean, std dev, min/max values for all metrics

3. **Performance Plots**: `sa_performance_plot_YYYYMMDD_HHMMSS.png`
   - 4-panel visualization showing:
     - Execution time vs number of drones
     - Algorithm success rate
     - Solution quality (final cost)
     - Convergence behavior (iterations)

## Expected Results

Based on the algorithm complexity, you should expect:

- **Linear to Quadratic Growth**: Execution time should increase with drone count
- **Higher Conflict Resolution**: More drones = more potential conflicts to resolve
- **Convergence Variation**: Different convergence patterns for different problem sizes
- **Success Rate**: Should remain high (>90%) for reasonable drone counts

## Troubleshooting

### Common Issues

1. **"Not enough available nodes"**: 
   - The graph has limited edge nodes for route generation
   - Reduce requested drone count or increase graph size

2. **"No path found"**: 
   - Graph connectivity issues due to random availability
   - Re-run test (different random seed) or adjust availability percentage

3. **Long execution times**: 
   - SA is computationally intensive for large drone counts
   - Consider reducing max iterations or adjusting SA parameters

### Performance Tuning

To modify SA parameters, edit the `simulated_annealing` method in `SimulatedAnnealingOptimizer`:

```python
# Current default parameters
initial_temp=1000,     # Starting temperature
cooling_rate=0.9,      # Temperature reduction rate
min_temp=0.001,        # Stopping temperature
max_iterations=None    # Optional iteration limit
```

## Technical Implementation Details

### Graph Building Process
1. Generate 3D lattice coordinates using `numpy.linspace`
2. Assign random availability (75% probability)
3. Calculate 3D geodesic distances for edge weights
4. Connect each node to valid neighbors (up to 26 connections)

### Route Generation Strategy
- Uses edge nodes (leftmost and rightmost) for origins/destinations
- Ensures path existence using NetworkX shortest path
- Avoids route conflicts by tracking used nodes

### SA Algorithm Features
- Initial solution using shortest paths
- Neighbor generation via path segment rerouting
- Conflict detection using spatial-temporal occupancy
- Temperature-based acceptance probability

### Statistical Analysis
- Multiple runs per configuration for reliability
- Standard statistical measures (mean, std dev, min/max)
- Success rate tracking
- Automated result aggregation and visualization

## Dependencies

Required Python packages:
- `networkx`: Graph operations
- `geopy`: Geographic distance calculations
- `numpy`: Numerical computations
- `matplotlib`: Plotting and visualization
- `pandas`: Data manipulation and CSV export
- `plotly`: Interactive 3D visualization (for original script)

Install dependencies:
```bash
pip install networkx geopy numpy matplotlib pandas plotly
```

## Future Enhancements

Potential framework improvements:
- Parallel test execution for faster results
- Additional SA parameter tuning experiments
- Comparison with other optimization algorithms
- Real-world airspace data integration
- Dynamic obstacle scenarios
- Multi-objective optimization metrics

---

**Note**: This framework is designed for research and performance evaluation. The SA algorithm parameters may need tuning for specific operational requirements.
