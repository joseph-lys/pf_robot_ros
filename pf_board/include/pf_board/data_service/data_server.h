#ifndef DATA_SERVICE_DATA_SERVICE_H
#define DATA_SERVICE_DATA_SERVICE_H

namespace pf_board
{
namespace data_service
{

class IStateControl
{
 public:
  
  virtual bool motorBoardFromState();
  virtual bool motorBoardToSate();
  virtual bool motorRosToState();
  virtual bool motorRosFromState();

  virtual bool iosBoardFromState();
  virtual bool iosRosToState();
  virtual bool iosRosToState();
  virtual bool iosRosFromState();
  
  virtual bool enableIoService();
  virtual bool disableIoService();
};


}  // namespace data_service
}  // namespace pf_board

#endif  // DATA_SERVICE_DATA_SERVICE_H