#!/usr/bin/env python3
"""
Simple script to run the SA performance tests.
Execute this script to start the experimental evaluation.
"""

from SA_Performance_Test import PerformanceTestRunner

def main():
    """Run the performance tests with default parameters."""
    print("Starting Simulated Annealing Performance Tests...")
    print("This will test drone counts from 10 to 100 in steps of 10")
    print("Each test will be run 3 times for statistical reliability")
    print("-" * 60)
    
    # Create and run the test
    test_runner = PerformanceTestRunner()
    test_runner.setup()
    
    # Run tests with smaller drone counts first for quicker initial results
    drone_counts = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    runs_per_test = 3
    
    test_runner.run_performance_tests(drone_counts, runs_per_test)
    test_runner.analyze_results()
    test_runner.save_results()
    test_runner.plot_results()

if __name__ == "__main__":
    main()
