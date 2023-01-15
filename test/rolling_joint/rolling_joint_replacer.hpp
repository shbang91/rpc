#pragma once
#include "test/rolling_joint/rolling_joint.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/tokenizer.hpp>
#include <unordered_map>

inline std::vector<std::string> tokenizePrefixes(std::string name) {
  boost::replace_all(name, "__", ",");
  std::vector<std::string> tokens;

  boost::char_separator<char> sep{","};
  boost::tokenizer<boost::char_separator<char>> tokenizer(name, sep);
  for (boost::tokenizer<boost::char_separator<char>>::iterator it =
           tokenizer.begin();
       it != tokenizer.end(); ++it) {
    std::string token(*it);
    tokens.push_back(token);
  }

  return tokens;
}

class RollingJointReplacer {
public:
  RollingJointReplacer() {}
  ~RollingJointReplacer() {}

  bool handles(dart::dynamics::BodyNode *node);

  dart::dynamics::BodyNode *replace(dart::dynamics::SkeletonPtr old_skeleton,
                                    dart::dynamics::SkeletonPtr new_skeleton,
                                    dart::dynamics::BodyNode *old_node,
                                    dart::dynamics::BodyNode *new_parent);

private:
  std::unordered_map<std::string, dart::dynamics::Joint *> m_mimic_joint_map;

  dart::dynamics::BodyNode *
  replaceL1withFixed(dart::dynamics::SkeletonPtr old_skeleton,
                     dart::dynamics::SkeletonPtr new_skeleton,
                     dart::dynamics::BodyNode *old_node,
                     dart::dynamics::BodyNode *new_parent);
};
