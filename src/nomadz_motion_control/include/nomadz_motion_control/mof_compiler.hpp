#pragma once

#include <cstddef>
#include <boost/bimap.hpp>

#include "nomadz_motion_control/joint_requests.hpp"
#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"

namespace nomadz_motion_control {
  class MofCompiler {
  private:
    static const int MAXLAB = 5000;
    static const int NUM_SPECIAL_ACTIONS =
      static_cast<int>(nomadz_motion_control_msgs::SpecialActionType::NUM_SPECIAL_ACTIONS);
    int num_of_labels_ = 0;
    char* label_motion_[MAXLAB];
    char* label_name_[MAXLAB];
    short label_number_[MAXLAB];

    static const int MAXLIN = 32000;
    int num_of_lines_ = 0;
    char motion_[512];
    int act_motion_id_ = -1;
    char* line_data_[MAXLIN];
    short line_number_[MAXLIN];
    short line_motion_id_[MAXLIN];

    static const int MAXFIL = 500;
    int num_of_files_ = 0;
    char* file_name_[MAXFIL];
    short file_start_index_[MAXFIL];

    int jump_table_[NUM_SPECIAL_ACTIONS];

    char* print_buffer_;
    size_t print_buffer_size_;
    std::string installation_root_directory_;

    using bm_type = boost::bimap<nomadz_motion_control_msgs::SpecialActionType, std::string>;
    using position = bm_type::value_type;
    bm_type mof_file_map_;

  public:
    MofCompiler();
    ~MofCompiler();
    int myprintf(const char* format, ...);
    bool generateMotionNet();
    bool parseExternMotionFile();
    bool parseMotionFiles();
    bool compileMotionFiles(char* buffer, size_t size);
  };
} // namespace nomadz_motion_control
