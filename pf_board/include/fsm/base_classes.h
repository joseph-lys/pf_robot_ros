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
struct BaseContext  
{
  virtual ~BaseContext()
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

  virtual AbstractState* receiveFromRos(BaseContext& internal_state, BaseContext& data_from_ros) = 0;

  virtual AbstractState* noReceiveFromRos(BaseContext& internal_state) = 0;
  
  virtual AbstractState* sendToBoard(BaseContext& internal_state, BaseContext& data_to_board) = 0; 

  virtual AbstractState* receiveFromBoard(BaseContext& internal_state, BaseContext& data_to_board) = 0;

  virtual AbstractState* noReceiveFromBoard(BaseContext& internal_state) = 0;

  virtual AbstractState* sendToRos(BaseContext& internal_state, BaseContext& data_to_board) = 0;

  virtual AbstractState* nextState(BaseContext& internal_state) = 0;

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

/// Composite state base class
/// All composite implementation shall inherit from this class
/// If there is no change of state, nullptr will be returned for each call

class BaseCompositeState : public AbstractState
{
 public:
  AbstractState* serviceCheckIOWrite(bool& service_available) override;
  AbstractState* serviceCheckIORead(bool& service_available) override;
  AbstractState* serviceCheckMotor(bool& service_available) override;
  AbstractState* receiveFromRos(BaseContext& internal_state, BaseContext& data_from_ros) override;
  AbstractState* noReceiveFromRos(BaseContext& internal_state) override;
  AbstractState* sendToBoard(BaseContext& internal_state, BaseContext& data_to_board) override; 
  AbstractState* receiveFromBoard(BaseContext& internal_state, BaseContext& data_from_board) override;
  AbstractState* noReceiveFromBoard(BaseContext& internal_state) override;
  AbstractState* sendToRos(BaseContext& internal_state, BaseContext& data_to_ros) override;
  AbstractState* nextState(BaseContext& internal_state) override;
  virtual void onEntry();  // cleanup / setup operation during entry into state

};

/// Component state base class
/// All actual states shall inherit from this class
///
class BaseComponentState : public AbstractState
{
 public:
  AbstractState* serviceCheckIOWrite(bool& service_available) final;
  AbstractState* serviceCheckIORead(bool& service_available) final;
  AbstractState* serviceCheckMotor(bool& service_available) final;
  AbstractState* receiveFromRos(BaseContext& internal_state, BaseContext& data_from_ros) final;
  AbstractState* noReceiveFromRos(BaseContext& internal_state) final;
  AbstractState* sendToBoard(BaseContext& internal_state, BaseContext& data_to_board) final; 
  AbstractState* receiveFromBoard(BaseContext& internal_state, BaseContext& data_from_board) final;
  AbstractState* noReceiveFromBoard(BaseContext& internal_state) final;
  AbstractState* sendToRos(BaseContext& internal_state, BaseContext& data_to_ros) final;
  AbstractState* nextState(BaseContext& internal_state) final;
 private:
  std::vector<std::shared_ptr<BaseCompositeState>> composites_;
 protected:
  void onEntry();  // cleanup / setup operation for all components during entry into state
  template <typename DerivedComposite>
  void addComposite()
  {
    composites_.emplace_back(std::make_shared<DerivedComposite>());
  }
  BaseComponentState();  // only subclass can create an instance
};


/// BaseState is a follows a singleton pattern
template <class T> 
class BaseState : public BaseComponentState
{
 public:
  static AbstractState* entry()
  {
    if(pSingleton == nullptr)
      pSingleton = new T{};
    pSingleton->onEntry();
    return pSingleton->basePtr(); 
  }

 protected:
  static T* pSingleton;
  BaseState(){}
 private:
  BaseState(BaseState const &) = delete;
  BaseState& operator= (BaseState const &) = delete; 
};

template <class T>
T* BaseState<T>::pSingleton = nullptr;


} // namespace fsm

} // namespace pf_board

#endif  // FSM_BASE_CLASSES_H
