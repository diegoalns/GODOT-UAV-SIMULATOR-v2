#!/usr/bin/env python3
"""
Quick test script to verify the SA performance testing framework works.
This runs a smaller test with fewer drones to validate the implementation.
"""

from SA_Performance_Test import PerformanceTestRunner

def main():
    """Run a quick validation test."""
    print("Running Quick Validation Test...")
    print("Testing with 5, 10, and 15 drones (2 runs each)")
    print("-" * 50)
    
    # Create and run the test
    test_runner = PerformanceTestRunner()
    test_runner.setup()
    
    # Run a smaller test first
    drone_counts = [5, 10, 15]  # Smaller test
    runs_per_test = 2  # Fewer runs for quick validation
    
    test_runner.run_performance_tests(drone_counts, runs_per_test)
    test_runner.analyze_results()
    test_runner.save_results("quick_test_results.csv")
    
    print("\nQuick test completed! If this works, you can run the full test.")

if __name__ == "__main__":
    main()
