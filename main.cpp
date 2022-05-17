#include <cmath>
#include <cstdint>
#include <vector>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

struct Point {
    double x;
    double y;
};

double distance(Point p1, Point p2) {
    return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
}

void readGraph(std::vector<std::vector<double>>& matrix, int N, std::vector<Point>& points) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i == j) {
                matrix[i][j] = 0;
            } else {
                matrix[i][j] = distance(points[i], points[j]);
            }
        }
    }
}


namespace operations_research {
struct DataModel {
    std::vector<std::vector<double>> distance_matrix;
    const int num_vehicles = 1;
    const RoutingIndexManager::NodeIndex depot{0};
};

//! @brief Print the solution.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void PrintSolution(const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
  // Inspect solution.
//  LOG(INFO) << "Objective: " << solution.ObjectiveValue() << " miles";
  int64_t index = routing.Start(0);
//  LOG(INFO) << "Route:";
  int64_t distance{0};
  std::stringstream route;
  while (routing.IsEnd(index) == false) {
    route << manager.IndexToNode(index).value() << " -> ";
    int64_t previous_index = index;
    index = solution.Value(routing.NextVar(index));
    distance += routing.GetArcCostForVehicle(previous_index, index, int64_t{0});
  }
 // LOG(INFO) << route.str() << manager.IndexToNode(index).value();
  std::cout << distance << std::endl;
//  LOG(INFO) << "";
//  LOG(INFO) << "Advanced usage:";
//  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

void Tsp(DataModel init_data) {
  // Instantiate the data problem.
  DataModel data = init_data;

  // Create Routing Index Manager
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);

  // Create Routing Model.
  RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager.IndexToNode(from_index).value();
        auto to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  // Solve the problem.
  const Assignment* solution = routing.SolveWithParameters(searchParameters);

  // Print solution on console.
  PrintSolution(manager, routing, *solution);
}

}  // namespace operations_research

int main(int argc, char** argv) {
    std::string path = "C:\Users\garmash\Desktop\TSPtests";
               auto it = fs::directory_iterator(path);
               std::vector<fs::path> array_path;
               copy_if(fs::begin(it), fs::end(it), std::back_inserter(array_path),
                   [](const auto& entry) {
                       return fs::is_regular_file(entry);
               });
    for (auto& p : array_path) {
        std::ifstream fin;
        fin.open(p.string());
        std::cout << p.string() << std::endl;
        int64_t N;
        fin >> N;
        operations_research::DataModel data;
        std::vector<Point> points(N);
            for (int i = 0; i < N; i++) {
                Point p;
                fin >> p.x >> p.y;
                points[i] = p;
            }
            std::vector<std::vector<double> > matrix(N, std::vector<double>(N));
            readGraph(matrix, N, points);
            data.distance_matrix = matrix;
            operations_research::Tsp(data);
        }
    return EXIT_SUCCESS;
}
