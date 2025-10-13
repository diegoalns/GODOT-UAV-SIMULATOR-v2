#!/usr/bin/env python3
"""
Demo script for the full SA performance test with reduced parameters for demonstration.
This runs a subset of the full test to show the complete workflow.
"""

from SA_Performance_Test import PerformanceTestRunner

def main():
    """Run a demonstration of the full performance test."""
    print("Running Demonstration Performance Test...")
    print("Testing with drone counts: 10, 30, 50, 70, 90")
    print("2 runs per test configuration")
    print("-" * 60)
    
    # Create and run the test
    test_runner = PerformanceTestRunner()
    test_runner.setup()
    
    # Run demonstration test with subset of drone counts
    drone_counts = [10, 30, 50, 70, 90]  # Representative subset
    runs_per_test = 2  # Fewer runs for demo
    
    test_runner.run_performance_tests(drone_counts, runs_per_test)
    test_runner.analyze_results()
    test_runner.save_results("demo_performance_results.csv")
    
    print("\nDemo completed! For the full test (10-100 step 10, 3 runs each):")
    print("Run: python run_performance_test.py")
    print("\nThis will take longer but provide comprehensive results.")

if __name__ == "__main__":
    main()
