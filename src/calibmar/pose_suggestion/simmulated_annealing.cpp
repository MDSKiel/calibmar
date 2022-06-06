#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <random>

#include <iostream>

namespace {

  // step with length temperature with direction uniformly at random
  template <typename generator>
  std::vector<double> NextFunction(const std::vector<double>& current_temp, const std::vector<double>& current_solution,
                                   const std::vector<double>& upper_bounds, const std::vector<double>& lower_bounds,
                                   generator& gen) {
    // random directional vector needs normal distribution for its coefficients
    // c.f. https://towardsdatascience.com/the-best-way-to-pick-a-unit-vector-7bd0cc54f9b
    std::normal_distribution<double> random_norm(0, 1);
    std::uniform_real_distribution<double> random_unif(0, 1);

    std::vector<double> next_solution(current_solution.size());
    double norm = 0;
    for (size_t i = 0; i < next_solution.size(); i++) {
      double rand = random_norm(gen);
      next_solution[i] = rand;
      norm += rand * rand;
    }

    norm = std::sqrt(norm);

    // normalize and multiply by temperature as step size & add to current solution
    for (size_t i = 0; i < next_solution.size(); i++) {
      next_solution[i] = next_solution[i] / norm * current_temp[i] + current_solution[i];
    }

    // check bounds and randomly project inside if exceeding
    for (size_t i = 0; i < next_solution.size(); i++) {
      if (next_solution[i] < lower_bounds[i]) {
        next_solution[i] = current_solution[i] + ((lower_bounds[i] - current_solution[i]) * random_unif(gen));
      }
      else if (next_solution[i] > upper_bounds[i]) {
        next_solution[i] = current_solution[i] + ((upper_bounds[i] - current_solution[i]) * random_unif(gen));
      }
    }

    return next_solution;
  }

  bool CheckRollingAverage(std::queue<double>& nums, double* current_sum, double new_value) {
    int avg_limit = 20;
    double avg_tolerance = 1e-6;

    new_value = std::abs(new_value);
    nums.push(new_value);

    if (nums.size() > avg_limit) {
      *current_sum += new_value - nums.front();
      nums.pop();

      double avg = *current_sum / avg_limit;
      return avg > avg_tolerance;
    }
    else {
      // less than limit values is always ok
      *current_sum += new_value;
      return true;
    }
  }
}

namespace calibmar {
  void SimulatedAnnealing(std::vector<double>& x, double& minf, const std::vector<double>& upper_bounds,
                          const std::vector<double>& lower_bounds, int max_iter,
                          const std::function<double(std::vector<double>&, void*)>& cost, void* cost_data,
                          const std::function<double(int, std::vector<double>&, void*)>& temperature, void* temp_data) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<double> random(0, 1);

    minf = std::numeric_limits<double>::max();
    double cost_old = cost(x, cost_data);
    double cost_new = cost_old, cost_prev = cost_old;
    std::vector<double> x_old = x;

    std::queue<double> rolling_avg_elems;
    double avg_sum = 0;
    std::vector<double> current_temp(x.size());

    for (int i = 0; i <= max_iter; i++) {
      double T = temperature(i, current_temp, temp_data);

      std::vector<double> x_new = NextFunction(current_temp, x, upper_bounds, lower_bounds, gen);
      cost_prev = cost_new;
      double tmp = cost(x_new, cost_data);
      if (tmp == std::numeric_limits<double>::max() || std::isnan(tmp)) {
        continue;
      }
      cost_new = tmp;

      // using "default" acceptance function e^(c_o-c_n/max(T))
      if (cost_new < cost_old || std::exp((cost_old - cost_new) / T) > random(gen)) {
        x_old = std::move(x_new);
        cost_old = cost_new;

        if (cost_new < minf) {
          // save if current best
          minf = cost_old;
          x = x_old;
        }
      }
      // Stop if the average change of the cost does not vary anymore
      if (!CheckRollingAverage(rolling_avg_elems, &avg_sum, cost_prev - cost_new)) {
        break;
      }
    }
  }
}