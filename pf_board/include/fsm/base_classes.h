/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_BASE_CLASSES_H
#define FSM_BASE_CLASSES_H

#include <memory>

namespace fsm
{
class BaseState;  // forward declaration
struct BaseContext  
{
   virtual ~BaseContext()
   {
   }
};

typedef std::shared_ptr<BaseState> StatePtr;
typedef std::shared_ptr<BaseContext> ContextPtr;

/// State base class
/// The functions will be called in this sequence
/// [some operations] -> receiveFromRos() -> sendToBoard() -> 
///  [transfer to board] -> 
/// receiveFromBoard -> sendToRos() -> [some operation]
///
class BaseState
{
 public:
  BaseState();
  virtual ~BaseState();

  virtual void receiveFromRos(ContextPtr);
  
  virtual void sendToBoard(ContextPtr); 

  virtual void receiveFromBoard(ContextPtr);

  virtual void sendToRos(ContextPtr);

  virtual std::shared_ptr<BaseState> nextState();

};



} // namespace fsm
#endif  // FSM_BASE_CLASSES_H
