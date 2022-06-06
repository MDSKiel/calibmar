#pragma once

#include <functional>

namespace calibmar {
  // Simmulated annealing optimization.
  //
  // The current temperature is used as step size in a random direction. Acceptance function is exp(cost_new-cost_old)/max(T)).
  // See e.g. Adaptive Simmulated Annealing https://doi.org/10.1007/978-3-642-27479-4_4.
  //
  // @param x Initial cost input value. Best solution output after optimization.
  // @param minf Lowest cost output.
  // @param upper_bounds Upper bounds for each solution. Must be of same dimension as x.
  // @param lower_bounds Lower bounds for each solution. Must be of same dimension as x.
  // @param max_iter Maximum iterations to run the cost function for.
  // @param cost Cost function. Must take x, cost_data and return the cost for respective x.
  // @param cost_data Additional data for cost function.
  // @param temperature Temperature function. Must take current iteration count, temperature_data and
  //        assign a temperature for the current iteration for each x. Should return the highest temperature among those.
  // @param temperature_data Additional data for temperature function.
  void SimulatedAnnealing(std::vector<double>& x, double& minf, const std::vector<double>& upper_bounds,
                          const std::vector<double>& lower_bounds, int max_iter,
                          const std::function<double(std::vector<double>&, void*)>& cost, void* cost_data,
                          const std::function<double(int, std::vector<double>&, void*)>& temperature, void* temp_data);
}