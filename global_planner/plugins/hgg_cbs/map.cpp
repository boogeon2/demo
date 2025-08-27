#include "map.h"

bool Map::get_map(std::string &map_file_name)
{
    YAML::Node map_file = YAML::LoadFile(map_file_name);
    const YAML::Node &vertices = map_file["levels"]["L1"]["vertices"];
    const YAML::Node &lanes = map_file["levels"]["L1"]["lanes"];
    int deleted_vertices = 0;
    // add nodes
    for (const auto &vertex : vertices)
    {
        if (vertex[3].as<std::string>() == "")
        {
            deleted_vertices++;
            continue;
        }
        gNode node;
        node.i = vertex[0].as<double>();
        node.j = vertex[1].as<double>();
        node.id = std::stoi(vertex[3].as<std::string>().substr(1)) - 1;
        nodes.push_back(node);
    }
    // add edges
    for (const auto &lane : lanes)
    {
        int id1 = lane[0].as<int>() - deleted_vertices;
        int id2 = lane[1].as<int>() - deleted_vertices;

        nodes[id1].neighbors.push_back(id2);
        edges.emplace_back(edges.size(), id1, id2);

        nodes[id2].neighbors.push_back(id1);
        edges.emplace_back(edges.size(), id2, id1);
    }
    for (gNode cur : nodes)
    {
        Node node;
        std::vector<Node> neighbors;
        neighbors.clear();
        for (unsigned int i = 0; i < cur.neighbors.size(); i++)
        {
            node.i = nodes[cur.neighbors[i]].i;
            node.j = nodes[cur.neighbors[i]].j;
            node.id = cur.neighbors[i];
            neighbors.push_back(node);
        }
        valid_moves.push_back(neighbors);
    }
    size = int(nodes.size());
    if (size == 0)
    {
        std::cout << "Error creating route_planner map!" << std::endl;
        return false;
    }
    return true;
}

Edge Map::get_edge(int entry, int exit) const
{
    for (std::size_t i = 0; i < edges.size(); i++)
    {
        if (edges[i].entry == entry and edges[i].exit == exit)
            return edges[i];
    }
    return Edge();
}

// Before changing obs_type ------------------------------------------------------------------------
bool Map::add_obstacle_info(const ObstacleInfo &obs_info)
{
    obstacles[obs_info.id].registered = true;
    return obstacle_info.insert({{obs_info.entry, obs_info.exit}, {obs_info}}).second;
}
// Before changing obs_type ------------------------------------------------------------------------

// After changing obs_type -------------------------------------------------------------------------
bool Map::add_obs_edge_info(const ObsEdgeInfo &obs_edge_info)
{
    for (auto obs : obs_edge_info.obstacles)
        obstacles[obs.id].registered = true;
    return obstacle_edge_info.insert({{obs_edge_info.entry, obs_edge_info.exit}, {obs_edge_info}}).second;
}
// After changing obs_type -------------------------------------------------------------------------

double Map::get_i(int id) const
{
    if (!map_is_roadmap)
        return int(id / width);
    else
        return nodes[id].i;
}

double Map::get_j(int id) const
{
    if (!map_is_roadmap)
        return int(id % width);
    else
        return nodes[id].j;
}

int Map::get_id(double i, double j) const
{
    if (!map_is_roadmap)
        return int(i * width + j);
    else
    {
        // 완벽히 같지 않아도 가장 가까운 i, j를 가지는 node의 id를 탐색 후 반환
        double min_dist = CN_INFINITY;
        int min_id = -1;
        for (gNode node : nodes)
        {
            double dist = sqrt(pow(node.i - i, 2) + pow(node.j - j, 2));
            if (dist < min_dist)
            {
                min_dist = dist;
                min_id = node.id;
            }
        }
        return min_id;
    }
    return -1;
}

bool Map::get_grid(const char *FileName)
{

    tinyxml2::XMLElement *root = nullptr, *map = nullptr, *element = nullptr, *mapnode = nullptr;

    std::string value;
    std::stringstream stream;
    bool hasGridMem(false), hasGrid(false), hasHeight(false), hasWidth(false);

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }
    map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "Error! No '" << CNS_TAG_MAP << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (mapnode = map->FirstChildElement(); mapnode; mapnode = mapnode->NextSiblingElement())
    {
        element = mapnode->ToElement();
        value = mapnode->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        stream.str("");
        stream.clear();
        stream << element->GetText();

        if (!hasGridMem and hasHeight and hasWidth)
        {
            grid.resize(height);
            for (int i = 0; i < height; ++i)
                grid[i].resize(width);
            hasGridMem = true;
        }

        if (value == CNS_TAG_HEIGHT)
        {
            if (hasHeight)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_HEIGHT << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_HEIGHT << "' =" << height << "will be used."
                          << std::endl;
            }
            else
            {
                if (!((stream >> height) and (height > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_HEIGHT
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_HEIGHT << "' tag should be an integer >=0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_HEIGHT
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                    hasHeight = true;
            }
        }
        else if (value == CNS_TAG_WIDTH)
        {
            if (hasWidth)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_WIDTH << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_WIDTH << "' =" << width << "will be used." << std::endl;
            }
            else
            {
                if (!((stream >> width) and (width > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_WIDTH
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_WIDTH << "' tag should be an integer AND >0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_WIDTH
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                    hasWidth = true;
            }
        }
        else if (value == CNS_TAG_GRID)
        {
            int grid_i(0), grid_j(0);
            hasGrid = true;
            if (!(hasHeight and hasWidth))
            {
                std::cout << "Error! No '" << CNS_TAG_WIDTH << "' tag or '" << CNS_TAG_HEIGHT << "' tag before '"
                          << CNS_TAG_GRID << "'tag encountered!" << std::endl;
                return false;
            }
            element = mapnode->FirstChildElement();
            while (grid_i < height)
            {
                if (!element)
                {
                    std::cout << "Error! Not enough '" << CNS_TAG_ROW << "' tags inside '" << CNS_TAG_GRID << "' tag."
                              << std::endl;
                    std::cout << "Number of '" << CNS_TAG_ROW
                              << "' tags should be equal (or greater) than the value of '" << CNS_TAG_HEIGHT
                              << "' tag which is " << height << std::endl;
                    return false;
                }
                std::string str = element->GetText();
                std::vector<std::string> elems;
                std::stringstream ss(str);
                std::string item;
                while (std::getline(ss, item, ' '))
                    elems.push_back(item);
                grid_j = 0;
                int val;
                if (elems.size() > 0)
                    for (grid_j = 0; grid_j < width; ++grid_j)
                    {
                        if (grid_j == int(elems.size()))
                            break;
                        stream.str("");
                        stream.clear();
                        stream << elems[grid_j];
                        stream >> val;
                        grid[grid_i][grid_j] = val;
                    }

                if (grid_j != width)
                {
                    std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << grid_i + 1 << " " << CNS_TAG_ROW
                              << std::endl;
                    return false;
                }
                ++grid_i;
                element = element->NextSiblingElement();
            }
        }
    }
    if (!hasGrid)
    {
        std::cout << "Error! There is no tag 'grid' in xml-file!\n";
        return false;
    }
    size = width * height;
    std::vector<Step> moves;
    valid_moves.resize(height * width);
    if (connectedness == 2)
        moves = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};
    else if (connectedness == 3)
        moves = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
    else if (connectedness == 4)
        moves = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {1, 2}, {2, 1}, {2, -1}, {1, -2}, {-1, -2}, {-2, -1}, {-2, 1}, {-1, 2}};
    else
        moves = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {1, 2}, {2, 1}, {2, -1}, {1, -2}, {-1, -2}, {-2, -1}, {-2, 1}, {-1, 2}, {1, 3}, {2, 3}, {3, 2}, {3, 1}, {3, -1}, {3, -2}, {2, -3}, {1, -3}, {-1, -3}, {-2, -3}, {-3, -2}, {-3, -1}, {-3, 1}, {-3, 2}, {-2, 3}, {-1, 3}};
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            std::vector<bool> valid(moves.size(), true);
            for (unsigned int k = 0; k < moves.size(); k++)
                if ((i + moves[k].i) < 0 or (i + moves[k].i) >= height or (j + moves[k].j) < 0 or (j + moves[k].j) >= width or cell_is_obstacle(i + moves[k].i, j + moves[k].j) or !check_line(i, j, i + moves[k].i, j + moves[k].j))
                    valid[k] = false;
            std::vector<Node> v_moves = {};
            for (unsigned int k = 0; k < valid.size(); k++)
                if (valid[k])
                    v_moves.push_back(Node((i + moves[k].i) * width + moves[k].j + j, 0, 0, i + moves[k].i, j + moves[k].j));
            valid_moves[i * width + j] = v_moves;
        }
    return true;
}

bool Map::get_roadmap(const char *FileName)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    tinyxml2::XMLElement *root = 0, *element = 0, *data;
    std::string value;
    std::stringstream stream;
    root = doc.FirstChildElement("graphml")->FirstChildElement("graph");
    for (element = root->FirstChildElement("node"); element; element = element->NextSiblingElement("node"))
    {
        data = element->FirstChildElement();

        stream.str("");
        stream.clear();
        stream << data->GetText();
        stream >> value;
        auto it = value.find_first_of(",");
        stream.str("");
        stream.clear();
        stream << value.substr(0, it);
        double i;
        stream >> i;
        stream.str("");
        stream.clear();
        value.erase(0, ++it);
        stream << value;
        double j;
        stream >> j;
        gNode node;
        node.i = i;
        node.j = j;
        node.id = nodes.size();
        nodes.push_back(node);
    }
    for (element = root->FirstChildElement("edge"); element; element = element->NextSiblingElement("edge"))
    {
        std::string source = std::string(element->Attribute("source"));
        std::string target = std::string(element->Attribute("target"));
        source.erase(source.begin(), ++source.begin());
        target.erase(target.begin(), ++target.begin());
        int id1, id2;
        stream.str("");
        stream.clear();
        stream << source;
        stream >> id1;
        stream.str("");
        stream.clear();
        stream << target;
        stream >> id2;
        nodes[id1].neighbors.push_back(id2);
        edges.emplace_back(edges.size(), id1, id2);
    }
    for (gNode cur : nodes)
    {
        Node node;
        std::vector<Node> neighbors;
        neighbors.clear();
        for (unsigned int i = 0; i < cur.neighbors.size(); i++)
        {
            node.i = nodes[cur.neighbors[i]].i;
            node.j = nodes[cur.neighbors[i]].j;
            node.id = cur.neighbors[i];
            if (node.id >= CN_DNODES)
                node.type = 0;
            else
                node.type = 1;
            neighbors.push_back(node);
        }
        valid_moves.push_back(neighbors);
    }
    size = int(nodes.size());
    return true;
}

void Map::print_map()
{
    std::cout << height << "x" << width << std::endl;
    for (int i = 0; i < height; i++)
    {
        std::cout << "<row>";
        for (int j = 0; j < width; j++)
            std::cout << grid[i][j] << " ";
        std::cout << "</row>" << std::endl;
    }
}

void Map::printPPM()
{
    std::cout << "P3\n"
              << width << " " << height << "\n255\n";
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            if (grid[i][j] == 1)
                std::cout << "0 0 0\n";
            else
                std::cout << "255 255 255\n";
        }
}

bool Map::cell_is_obstacle(int i, int j) const
{
    return (grid[i][j] == CN_OBSTL);
}

std::vector<Node> Map::get_valid_moves(int id) const
{
    return valid_moves[id];
}

bool Map::check_line(int x1, int y1, int x2, int y2)
{
    int delta_x(std::abs(x1 - x2));
    int delta_y(std::abs(y1 - y2));
    if ((delta_x > delta_y and x1 > x2) or (delta_y >= delta_x and y1 > y2))
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    int step_x(x1 < x2 ? 1 : -1);
    int step_y(y1 < y2 ? 1 : -1);
    int error(0), x(x1), y(y1);
    int gap = int(agent_size * sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y) / 2 - CN_EPSILON);
    int k, num;

    if (delta_x > delta_y)
    {
        int extraCheck = int(agent_size * delta_y / sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON);
        for (int n = 1; n <= extraCheck; n++)
        {
            error += delta_y;
            num = (gap - error) / delta_x;
            for (k = 1; k <= num; k++)
                if (cell_is_obstacle(x1 - n * step_x, y1 + k * step_y))
                    return false;
            for (k = 1; k <= num; k++)
                if (cell_is_obstacle(x2 + n * step_x, y2 - k * step_y))
                    return false;
        }
        error = 0;
        for (x = x1; x != x2 + step_x; x++)
        {
            if (cell_is_obstacle(x, y))
                return false;
            if (x < x2 - extraCheck)
            {
                num = (gap + error) / delta_x;
                for (k = 1; k <= num; k++)
                    if (cell_is_obstacle(x, y + k * step_y))
                        return false;
            }
            if (x > x1 + extraCheck)
            {
                num = (gap - error) / delta_x;
                for (k = 1; k <= num; k++)
                    if (cell_is_obstacle(x, y - k * step_y))
                        return false;
            }
            error += delta_y;
            if ((error << 1) > delta_x)
            {
                y += step_y;
                error -= delta_x;
            }
        }
    }
    else
    {
        int extraCheck = int(agent_size * delta_x / sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON);
        for (int n = 1; n <= extraCheck; n++)
        {
            error += delta_x;
            num = (gap - error) / delta_y;
            for (k = 1; k <= num; k++)
                if (cell_is_obstacle(x1 + k * step_x, y1 - n * step_y))
                    return false;
            for (k = 1; k <= num; k++)
                if (cell_is_obstacle(x2 - k * step_x, y2 + n * step_y))
                    return false;
        }
        error = 0;
        for (y = y1; y != y2 + step_y; y += step_y)
        {
            if (cell_is_obstacle(x, y))
                return false;
            if (y < y2 - extraCheck)
            {
                num = (gap + error) / delta_y;
                for (k = 1; k <= num; k++)
                    if (cell_is_obstacle(x + k * step_x, y))
                        return false;
            }
            if (y > y1 + extraCheck)
            {
                num = (gap - error) / delta_y;
                for (k = 1; k <= num; k++)
                    if (cell_is_obstacle(x - k * step_x, y))
                        return false;
            }
            error += delta_x;
            if ((error << 1) > delta_y)
            {
                x += step_x;
                error -= delta_y;
            }
        }
    }
    return true;
}
