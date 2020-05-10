/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_COMMS_SPI_DRIVER_H_
#define PF_BOARD_COMMS_SPI_DRIVER_H_
namespace pf_board {
namespace comms {

class SpiDriver
{
 public:
  SpiDriver() = default;
  ~SpiDriver();

  /// Set SPI Mode
  /// @param mode SPI mode, see https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Mode_numbers
  bool setMode(uint32_t mode);

  /// Set delay between CS and first clock trigger
  /// @param delay_us delay in microseconds
  bool setDelay(uint16_t delay_us);

  /// Set Clock Frequency
  /// @param freq_hz SPI clock frequency in Hertz
  bool setClockFrequency(uint32_t freq_hz);

  /// Set to keep file descriptor open
  /// @param keep_open true: does not close descriptor, false: open and close descriptor every use
  bool setKeepOpen(bool keep_open)

  /// Set device name
  /// @param dev string of SPI dev, such as "/dev/spidev0.0"
  bool setDeviceName(std::string dev);

  bool transfer(uint8_t* tx_buffer, uint8_t rx_buffer, uint32_t length);

 private:
  bool openFd();
  bool closeFd();

  bool keep_open_ = false;
  uint32_t delay_us_ = 0;
  uint16_t freq_hz_ = 0;
  int fd_ = 0;
  std::string = dev_;
};

}  // namespace comms
}  // namespace pf_board

#endif  // PF_BOARD_COMMS_SPI_DRIVER_H_