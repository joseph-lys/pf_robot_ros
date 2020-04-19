/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_BASE_CLASSES_H
#define FSM_BASE_CLASSES_H

#include <vector>
#include <memory>

namespace pf_board
{
namespace fsm
{
struct StateData  
{
  virtual ~StateData()
  {
  }
};

struct DataFromRos
{
  virtual ~DataFromRos()
  {
  }
};
struct DataToBoard
{
  virtual ~DataToBoard()
  {
  }
};
struct DataFromBoard
{
  virtual ~DataFromBoard()
  {
  }
};
struct DataToRos
{
  virtual ~DataToRos()
  {
  }
};

/// State base class
/// The functions will be called in this sequence
/// [some operations] -> receiveFromRos() -> sendToBoard() -> [transfer to board] ->  receiveFromBoard -> sendToRos() -> [some operation]
///
class AbstractState
{
 public:
  AbstractState() = default;
  virtual ~AbstractState() = default;

  virtual AbstractState* serviceCheckIOWrite(bool& service_available) = 0;

  virtual AbstractState* serviceCheckIORead(bool& service_available) = 0;

  virtual AbstractState* serviceCheckMotor(bool& service_available) = 0;

  virtual AbstractState* receiveFromRos(StateData& internal_state, DataFromRos& data_from_ros) = 0;

  virtual AbstractState* noReceiveFromRos(StateData& internal_state) = 0;
  
  virtual AbstractState* sendToBoard(StateData& internal_state, DataToBoard& data_to_board) = 0; 

  virtual AbstractState* receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_board) = 0;

  virtual AbstractState* noReceiveFromBoard(StateData& internal_state) = 0;

  virtual AbstractState* sendToRos(StateData& internal_state, DataToRos& data_to_ros) = 0;

  virtual AbstractState* nextState(StateData& internal_state) = 0;

 protected:
  template<typename T>
  constexpr T& derivedRef() 
  {
    return dynamic_cast<T&>(*this);
  }
  template<typename T>
  constexpr T& derivedPtr() 
  {
    return dynamic_cast<T*>(this);
  }
  constexpr AbstractState& baseRef() 
  {
    return static_cast<AbstractState&>(*this);
  }
  constexpr AbstractState* basePtr() 
  {
    return static_cast<AbstractState*>(this);
  }
};

/// State Decorator base class
/// If there is no change of state, nullptr will be returned for each call

class BaseStateDecorator : public AbstractState
{
 public:
  AbstractState* serviceCheckIOWrite(bool& service_available) override;
  AbstractState* serviceCheckIORead(bool& service_available) override;
  AbstractState* serviceCheckMotor(bool& service_available) override;
  AbstractState* receiveFromRos(StateData& internal_state, DataFromRos& data_from_ros) override;
  AbstractState* noReceiveFromRos(StateData& internal_state) override;
  AbstractState* sendToBoard(StateData& internal_state, DataToBoard& data_to_board) override; 
  AbstractState* receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_board) override;
  AbstractState* noReceiveFromBoard(StateData& internal_state) override;
  AbstractState* sendToRos(StateData& internal_state, DataToRos& data_to_ros) override;
  AbstractState* nextState(StateData& internal_state) override;
  virtual void onEntry();  // cleanup / setup operation during entry into state
  BaseStateDecorator(uint64_t timer_duration_ns);
 protected:
  /// Access to internal timer
  bool timerReached();
  void resetTimer();
 private:
  uint64_t timer_duration_ns_;
};

/// Component state base class
/// All actual states shall inherit from this class
///
class BaseStatePrototype : public AbstractState
{
 public:
  AbstractState* serviceCheckIOWrite(bool& service_available) final;
  AbstractState* serviceCheckIORead(bool& service_available) final;
  AbstractState* serviceCheckMotor(bool& service_available) final;
  AbstractState* receiveFromRos(StateData& internal_state, DataFromRos& data_from_ros) final;
  AbstractState* noReceiveFromRos(StateData& internal_state) final;
  AbstractState* sendToBoard(StateData& internal_state, DataToBoard& data_to_board) final; 
  AbstractState* receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_board) final;
  AbstractState* noReceiveFromBoard(StateData& internal_state) final;
  AbstractState* sendToRos(StateData& internal_state, DataToRos& data_to_ros) final;
  AbstractState* nextState(StateData& internal_state) final;
 private:
  std::vector<std::shared_ptr<BaseStateDecorator>> composites_;
 protected:
  void onEntry();  // cleanup / setup operation for all components during entry into state
  template <typename Decorator>
  void addDecorator()
  {
    composites_.emplace_back(std::make_shared<Decorator>());
  }
  BaseStatePrototype();  // only subclass can create an instance
};


/// BaseState is a follows a singleton pattern
template <class T> 
class BaseState : public BaseStatePrototype
{
 public:
  static AbstractState* entry()
  {
    if(p_singleton_ == nullptr)
      p_singleton_ = new T{};
    p_singleton_->onEntry();
    return p_singleton_->basePtr(); 
  }
  ~BaseState()
  {
    if (p_singleton_ != nullptr)
      delete p_singleton_;
  }
 protected:
  static T* p_singleton_;
  BaseState(){}
 private:
  BaseState(BaseState const &) = delete;
  BaseState& operator= (BaseState const &) = delete; 
};

template <class T>
T* BaseState<T>::p_singleton_ = nullptr;


} // namespace fsm
} // namespace pf_board

#endif  // FSM_BASE_CLASSES_H
