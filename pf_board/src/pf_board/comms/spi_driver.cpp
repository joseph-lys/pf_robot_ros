/// Copyright 2020 Joseph Lee Yuan Sheng
///
#include "pf_board/comms/spi_driver.h"

#include <string.h>

#include <fcntl.h>
#include <stdexcept>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>


using pf_board::comms::SpiDriver;

SpiDriver::~SpiDriver()
{
  closeFd();
}

bool SpiDriver::setMode(uint32_t mode)
{
  bool success = false;
  __u32 spi_mode;
  if (fd == 0 )
  {
    openFd();
  }
  if (fd > 0)
  {
    switch (mode) {
      case 0:
        spi_mode = SPI_MODE_0;
        break;
      case 1:
        spi_mode = SPI_MODE_1;
        break;
      case 2:
        spi_mode = SPI_MODE_2;
        break;
      case 3:
        spi_mode = SPI_MODE_3;
        break;
      default:
        throw std::invalid_argument("SPI invalid mode argument");
    }
    success = ioctl(fd, SPI_IOC_WR_MODE32, &spi_mode) >= 0;
    if (!keep_open_)
    {
      closeFd();
    }
  }
  return success;
}

bool SpiDriver::setClockFrequency(uint32_t freq_hz)
{
  bool success = false;
  __u32 spd = static_cast<__u32>(freq_hz);
  if (fd == 0 )
  {
    openFd();
  }
  if (fd > 0)
  {
    success = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spd) >= 0;
    if (!keep_open_)
    {
      closeFd();
    }
  }
  if (success)
  {
    freq_hz_ = freq_hz;
  }
  return success;
}

void SpiDriver::setKeepOpen(bool keep_open)
  keep_open_ = keep_open;
}

void SpiDriver::setDeviceName(std::string dev)
{
  dev_ = dev;
}

bool SpiDriver::transfer(uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length);
{
  bool success = false;
  __u32 spd = static_cast<__u32>(freq_hz);
  if (fd == 0 )
  {
    openFd();
  }
  if (fd > 0)
  {
    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof xfer);
    xfer.tx_buf = reinterpret_cast<__u64>(tx_buffer);
    xfer.rx_buf = reinterpret_cast<__u64>(rx_buffer);
    xfer.len = static_cast<__u32>(length);
    xfer.speed_hz = static_cast<__u32>(freq_hz_);
    xfer.delay_usecs = static_cast<__u16>(delay_us_);
    success = ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer) >= 0;
    if (!keep_open_)
    {
      closeFd();
    }
  }
  return success;
}

void SpiDriver::openFd()
{
  if (dev_.length()) {
    fd_ = open(dev_.c_str(), O_RDWR);
  }
}

void SpiDriver::closeFd()
{
  if (fd_) {
    close(fd_);
    fd_ = 0;
  }
}

