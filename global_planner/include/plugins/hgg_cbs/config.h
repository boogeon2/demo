#ifndef CONFIG_H
#define CONFIG_H
#include "const.h"
#include "string"
#include "iostream"
#include "sstream"
#include "math.h"
#include "yaml-cpp/yaml.h"

class Config
{
public:
    Config();
    void getConfig(const char *fileName);
    double precision;
    double focal_weight;
    bool use_cardinal;
    bool use_disjoint_splitting;
    int hlh_type;
    int connectedness;
    double agent_size;
    double timelimit;
    int num_dnodes;
    double lane_width;
    double trVel;
    double rotVel;
    double maxDiffInterval;
    double turn_radi;
    double lane_change_delay;
    double marginal_distance_to_avoid;
    double time_to_reach_centerline;
    double clearance_at_centerline;
    double safe_time;
};

#endif // CONFIG_H
