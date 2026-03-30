#include "analysis.h"
#include "tf.h"
#include "tf2.h"
using std::vector;
using std::pair;
using geometry_msgs::Point;

Analysis::Analysis(std::vector<Point> goals) :
    goals_(goals)
{
}

vector<double> Analysis::timeToImpact(Pose origin){
    vector<double> times;

    for (const auto& goal : goals_) {
        RangeBearingStamped rb = tf2::global2local(goal, origin);

        //rotate on the spot to face target
        double t_turn = std::abs(rb.bearing) / Display::OMEGA_MAX;
        //fly straight at max speed
        double t_fly  = rb.range / Display::V_MAX;

        times.push_back(t_turn + t_fly);
    }
    return times;
}

AdjacencyList Analysis::exportGraph(){
    AdjacencyList graph;

    // One entry per node
    graph.resize(goals_.size());

    for (unsigned int i = 0; i < goals_.size(); ++i) {
        for (unsigned int j = 0; j < goals_.size(); ++j) {
            if (i == j) continue;

            double dx   = goals_[j].x - goals_[i].x;
            double dy   = goals_[j].y - goals_[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            graph[i].push_back(std::make_pair(dist, j));
        }
    }
    return graph;
}