

## State Transitions
![State Transition Diagram](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/joseph-lys/pf_robot_ros/master/pf_board/include/pf_board/states/state_diagram.plantuml)

| State | OutputService | CommandService | MotorEnableService | PeriodicCommand |
| --- |--- | --- | --- |---|
|  Wait | NO<sup>1</sup> | NO | NO | NO
|  Sync | NO | NO | NO | NO
|  Stop | YES<sup>1</sup> | YES | YES <sup>2</sup>| YES <sup>3</sup>
|  Run  | YES | YES | YES <sup>3</sup>| YES
| Error | NO  | YES | NO | NO
| NonRecoverable | NO  | NO | NO | NO
_<sup>1</sup>All outputs are initialized to "safe state" when entering this state_  
_<sup>2</sup>transitions to Run if Motors Enabled_  
_<sup>3</sup>Motors will receive commands but does not move since torque diabled_  
_<sup>4</sup>transitions to Run if Motors Disabled_  



