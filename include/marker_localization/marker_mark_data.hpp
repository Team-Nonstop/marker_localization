#include <sstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#define VACANT  std::numeric_limits<unsigned int>::max()

class MarkerMarkData
{
public:
  class MarkData
  {
  public:
    unsigned int           idx;          /**< Index; assigned according to the order on YAML file */
    std::string            marker_id;
    std::string            marker_link;   /**< Short description (optional) */

    MarkData(unsigned int idx) : idx(idx) {};

    void operator << (const YAML::Node& node);
  };

  MarkerMarkData() : allowed(VACANT) { }
  ~MarkerMarkData() { }

  std::vector<MarkData>::size_type size() { return list.size(); };
  MarkData& operator [] (unsigned int idx) { return list[idx]; };

  void configure(const YAML::Node& node);

  unsigned int allowed;

private:
  std::vector<MarkData> list;
};
