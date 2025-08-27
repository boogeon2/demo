#include "config.h"
Config::Config()
{
    connectedness = CN_CONNECTEDNESS;
    use_cardinal = CN_USE_CARDINAL;
    agent_size = CN_AGENT_SIZE;
    timelimit = CN_TIMELIMIT;
    focal_weight = CN_FOCAL_WEIGHT;
    precision = CN_PRECISION;
    hlh_type = 0;
    num_dnodes = CN_DNODES;
    lane_width = CN_LANE;
    trVel = CN_TRVEL;
    rotVel = CN_ROTVEL;
    maxDiffInterval = CN_MAXDIFF;
    turn_radi = CN_TURNRADI;
    lane_change_delay = CN_LANECHANGE;
    marginal_distance_to_avoid = CN_AVOIDCLERANCE;
    time_to_reach_centerline = CN_CROSSTIME;
    clearance_at_centerline = CN_CROSSCLEARANCE;
    safe_time = CN_AGENT_SIZE * 2 / CN_TRVEL;
}

void Config::getConfig(const char *fileName)
{
    YAML::Node config_file = YAML::LoadFile(fileName);
    if (!config_file)
    {
        std::cout << "Error! No config file found." << std::endl;
        return;
    }

    YAML::Node config = config_file["/**"]["ros__parameters"]["global_planner"];
    YAML::Node algorithm = config[CNS_TAG_ALGORITHM];
    if (!algorithm)
    {
        std::cout << "No 'algorithm' element found in YAML file." << std::endl;
        return;
    }

    YAML::Node element = algorithm["precision"];
    if (!element)
    {
        std::cout << "Error! No 'precision' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_PRECISION << "'." << std::endl;
        precision = CN_PRECISION;
    }
    else
    {
        precision = element.as<double>();
        if (precision > 1.0 or precision <= 0)
        {
            std::cout << "Error! Wrong 'precision' value found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_PRECISION << "'." << std::endl;
            precision = CN_PRECISION;
        }
    }

    element = algorithm["use_cardinal"];
    if (!element)
    {
        std::cout << "Error! No 'use_cardinal' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_USE_CARDINAL << "'." << std::endl;
        use_cardinal = CN_USE_CARDINAL;
    }
    else
        use_cardinal = element.as<bool>();

    element = algorithm["use_disjoint_splitting"];
    if (!element)
    {
        std::cout << "Error! No 'use_disjoint_splitting' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_USE_DS << "'." << std::endl;
        use_disjoint_splitting = CN_USE_DS;
    }
    else
    {
        use_disjoint_splitting = element.as<bool>();
    }

    element = algorithm["connectedness"];
    if (!element)
    {
        std::cout << "Error! No 'connectedness' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_CONNECTEDNESS << "'." << std::endl;
        connectedness = CN_CONNECTEDNESS;
    }
    else
    {
        connectedness = element.as<int>();
        if (connectedness > 5 or connectedness < 2)
        {
            std::cout << "Error! Wrong 'connectedness' value found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_CONNECTEDNESS << "'." << std::endl;
            connectedness = CN_CONNECTEDNESS;
        }
    }

    element = algorithm["focal_weight"];
    if (!element)
    {
        std::cout << "Error! No 'focal_weight' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_FOCAL_WEIGHT << "'." << std::endl;
        focal_weight = CN_FOCAL_WEIGHT;
    }
    else
    {
        focal_weight = element.as<double>();
        if (focal_weight < 1.0)
        {
            std::cout << "Error! Wrong 'focal_weight' value found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_FOCAL_WEIGHT << "'." << std::endl;
            focal_weight = CN_FOCAL_WEIGHT;
        }
    }

    element = algorithm["agent_size"];
    if (!element)
    {
        std::cout << "Error! No 'agent_size' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_AGENT_SIZE << "'." << std::endl;
        agent_size = CN_AGENT_SIZE;
    }
    else
    {
        agent_size = element.as<double>();
        if (agent_size < 0 or agent_size > 0.5)
        {
            std::cout << "Error! Wrong 'agent_size' value found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_AGENT_SIZE << "'." << std::endl;
            agent_size = CN_AGENT_SIZE;
        }
    }

    element = algorithm["hlh_type"];
    if (!element)
    {
        std::cout << "Error! No 'hlh_type' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_HLH_TYPE << "'." << std::endl;
        hlh_type = CN_HLH_TYPE;
    }
    else
    {
        hlh_type = element.as<int>();
        if (hlh_type < 0 or hlh_type > 2)
        {
            std::cout << "Error! Wrong 'hlh_type' value found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_HLH_TYPE << "'." << std::endl;
            hlh_type = CN_HLH_TYPE;
        }
    }

    element = algorithm["timelimit"];
    if (!element)
    {
        std::cout << "Error! No 'timelimit' element found inside '" << CNS_TAG_ALGORITHM << "' section. It's compared to '" << CN_TIMELIMIT << "'." << std::endl;
        timelimit = CN_TIMELIMIT;
    }
    else
    {
        timelimit = element.as<double>();
        if (timelimit <= 0)
            timelimit = CN_INFINITY;
    }
    return;
}
