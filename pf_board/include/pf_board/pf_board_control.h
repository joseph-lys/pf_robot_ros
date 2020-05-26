/// Copyright 2020 Joseph Lee Yuan Sheng
///
#ifndef PF_BOARD_PF_BOARD_CONTROL_H
#define PF_BOARD_PF_BOARD_CONTROL_H


#include <unordered_map>

#include "pf_board/states/icontrol.h"
#include "pf_board/comms/spi_driver.h"
#include "pf_board/comms/transport_layer.h"
#include "control_msgs/JointJog.h"

namespace pf_board
{

class PfBoardControl : public states::IControl
{
 public:
  PfBoardControl() = delete;
  explicit PfBoardControl();

  /// Ros Message gets attached here.
  void rosReceiverCallback(const control_msgs::JointJog::ConstPtr& msg);

  ///
  void setBoardDriver(comms::SpiDriver* driver);

  /// 
  void setMapping(const std::vector<std::string>& joint_names,
                  const std::vector<uint8_t>& motor_ids);

  bool processDataFromRos() override;

  bool transferBoard(bool send_position, bool torque_enable) override;

  void processDataFromBoard() override;

  void transferDataToRos() override;

 private:
  size_t init_flags_ = 0;
  bool isInitialized();

  std::vector<std::string> joint_names_{};
  std::vector<uint8_t> motor_ids_{};
  std::vector<double> angle_adjustment_{};
  std::unordered_map<std::string, uint8_t> joint_names_2_index_;
  std::unordered_map<uint8_t, uint8_t> motor_id_2_index_;

  pf_board::comms::SpiDriver* p_driver_ = nullptr;
  control_msgs::JointJog::ConstPtr joint_jog_ = {};  /// empty shared_ptr

  bool has_control_data_ = false;
  bool has_feedback_data_ = false;

  uint8_t buffer_[256];
  pf_board::comms::TransportLayer transport_;

};


}  // namespace pf_board


#endif  // PF_BOARD_PF_BOARD_CONTROL_H
