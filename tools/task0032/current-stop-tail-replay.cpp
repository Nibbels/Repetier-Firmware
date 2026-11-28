#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <cstdio>
#include <algorithm>
#define RF_THERMAL_EMERGENCY_EXPERIMENT 1
#include <initializer_list>
#include "Constants.h"
#define INLINE inline
using flag8_t=uint8_t;using fast8_t=uint8_t;using speed_t=uint16_t;
enum {Z_MIN_PIN,Z_MAX_PIN,Z_DIR_PIN,Z_STEP_PIN};
constexpr bool INVERT_Z_DIR=false,LOW=false,HIGH=true;
constexpr float Z_ENDSTOP_DRIVE_OVER=0.25f;
constexpr uint16_t XYZ_STEPPER_HIGH_DELAY=100,XYZ_STEPPER_LOW_DELAY=100;
#ifndef MAX_STEP_PACKING_MIN_INTERVAL
#define MAX_STEP_PACKING_MIN_INTERVAL 5000
#endif
bool interruptsEnabled=true, high[3]={},positive[3]={};
bool closed[3]={},forceMin[3]={};int32_t physical[3]={};unsigned pulses=0;
bool scanRunning=false,queueOccupied=false,driverEnabled=true;
constexpr int32_t upper=1000;
class InterruptProtectedBlock { const bool previous_=interruptsEnabled; public:
 InterruptProtectedBlock(){interruptsEnabled=false;} ~InterruptProtectedBlock(){unprotect();} void unprotect(){interruptsEnabled=previous_;}
};
bool minContact(uint8_t axis){if(forceMin[axis])return true;if(physical[axis]<=0)closed[axis]=true;if(physical[axis]>=24)closed[axis]=false;return closed[axis];}
void pulse(uint8_t axis){assert(!high[axis]);high[axis]=true;physical[axis]+=positive[axis]?1:-1;++pulses;}
bool READ(int pin){if(pin==Z_DIR_PIN)return positive[Z_AXIS];if(pin==Z_MIN_PIN)return !minContact(Z_AXIS);assert(pin==Z_MAX_PIN);return physical[Z_AXIS]<upper;}
void WRITE(int pin,bool value){if(pin==Z_DIR_PIN){assert(!high[Z_AXIS]);positive[Z_AXIS]=value;}else{assert(pin==Z_STEP_PIN);if(value)pulse(Z_AXIS);else high[Z_AXIS]=false;}}
void maybeStop();
namespace HAL {
void allowInterrupts(){interruptsEnabled=true;maybeStop();}
void forbidInterrupts(){interruptsEnabled=false;}
void delayMicroseconds(int){assert(false);}
uint32_t timeInMilliseconds(){return 0;}
uint32_t CPUDivU2(uint16_t v){assert(v);return 16000000UL/v;}
uint16_t ComputeV(uint32_t,uint32_t){return 0;}
}
#include "XYMotion.h"
#include "ZMotion.h"
#include "CalibrationScan.h"
namespace Calibration {ScanSession scanSession;}
#define private public
#include "motion.h"
#include "StepExecutor.h"
#undef private
class MoveQueue{public:unsigned count()const{return queueOccupied;}void retireFront(){assert(false);}};
MoveQueue PrintLine::queue;PrintLine PrintLine::direct;
uint8_t PrintLine::queuedMoveCount(){return queue.count();}
StepExecutor::StepClock StepExecutor::stepClocks[2];StepExecutor::StepErrors StepExecutor::stepErrors[2];
StepExecutor::DirectionOwner StepExecutor::directionOwner=DirectionOwner::None;
volatile uint32_t StepExecutor::stepIntervalTicks=10000;
volatile uint16_t StepExecutor::packingInterval_=5000;
uint8_t StepExecutor::stepsPerTimerCall=1;
volatile int32_t StepExecutor::queuedPrimarySteps;int32_t StepExecutor::directPrimarySteps;
volatile float StepExecutor::nominalFeedrate_=0;
namespace Printer {
XYMotion xyMotion;ZMotion zMotion(100);
bool blockAll=false,homed=true;
struct {bool blocked=false,thermal=false;bool commandsBlocked()const{return blocked||thermal;}bool emergencyLocked()const{return thermal;}}printJob;
struct {void schedule(uint32_t){}}idleStatus;
struct {bool blocked()const{return false;}bool advanceEnabled()const{return false;}void addSteps(int){assert(false);}}extrusionOutput;
struct {bool addBaseStep(bool){assert(false);return false;}void consumeExtraStep(bool){assert(false);}}extrusionCompensation;
struct {void recordDirectExtrusion(int8_t){assert(false);}bool blocksCalibration()const{return false;}}pause;
struct {uint16_t intervalScale(){return 0;}}digitCompensation;
bool isAxisHomed(uint8_t){return homed;}bool zStepperEnabled(){return driverEnabled;}
bool needsCMPwait(){return false;}float stepsPerMm(uint8_t axis){assert(axis==Z_AXIS);return 100;}
float axisMaxFeedrate(uint8_t axis){assert(axis==Z_AXIS);return 40;}
bool isMinEndstopHit(uint8_t axis){return minContact(axis);}bool isMaxEndstopHit(uint8_t axis){return physical[axis]>=upper;}
bool getXDirectionIsPos(){return positive[X_AXIS];}bool getYDirectionIsPos(){return positive[Y_AXIS];}
void startXStep(int8_t d){pulse(X_AXIS);xyMotion.recordMotorStep(X_AXIS,d);}void startYStep(int8_t d){pulse(Y_AXIS);xyMotion.recordMotorStep(Y_AXIS,d);}
void endXYZSteps(){for(auto& h:high)h=false;}
void stopDirectAxis(uint8_t axis){if(axis<Z_AXIS)xyMotion.stopDirect(axis);else if(axis==Z_AXIS)zMotion.stopDirectMove();}
}
struct Extruder {int stepperDirection=0;static Extruder* current;static void step(){assert(false);}static void unstep(){}};
Extruder extruder;Extruder* Extruder::current=&extruder;
bool isAnyScanRunning(){return scanRunning;}
bool hasQueuedOvertravel(){return Printer::xyMotion.hasQueuedOvertravel()||Printer::zMotion.hasQueuedOvertravel();}
bool StepExecutor::enableSteppers(PrintLine&){assert(!scanRunning);return !Printer::blockAll&&!Printer::printJob.thermal;}
void StepExecutor::adjustDirections(PrintLine& m){if(Printer::blockAll)return;for(uint8_t axis=X_AXIS;axis<=Z_AXIS;++axis)if(m.isMoveOfAxis(axis))positive[axis]=m.isPositiveMoveOfAxis(axis);}
void StepExecutor::updateAdvanceSteps(const PrintLine&,uint16_t){}
void StepExecutor::stepSlowedZCompensation(){assert(!Printer::zMotion.correctionCanMove());}
namespace RMath{using std::min;}
#include "actual.inc"
static bool armed=false,requested=false;static unsigned visits=0,injectAt=0,pulsesAt=0;
static int32_t xAt=0;
void maybeStop(){
 if(!armed)return;
 assert(interruptsEnabled);
 if(++visits==injectAt){
  auto& m=PrintLine::direct;
  if(m.task!=DIRECT_RUNNING_STOPPABLE&&m.task!=DIRECT_RUNNING)return;
  InterruptProtectedBlock irq;const auto before=m.stepsRemaining;pulsesAt=pulses;xAt=physical[X_AXIS];
  PrintLine::stopDirectMove();assert(m.stepsRemaining==before);
  PrintLine::stopDirectMove();assert(m.stepsRemaining==before);
  requested=true;
 }
}
int32_t motor(uint8_t a){return a==Z_AXIS?Printer::zMotion.motorSteps():Printer::xyMotion.motorSteps(a);}
int32_t nominal(uint8_t a){return a==Z_AXIS?Printer::zMotion.nominalSteps():Printer::xyMotion.nominalSteps(a);}
const DirectPosition& ledger(uint8_t a){return a==Z_AXIS?Printer::zMotion.directPosition():Printer::xyMotion.directPosition(a);}
void reset(uint8_t a,int32_t origin){
 for(uint8_t i=0;i<3;++i){physical[i]=i==a?origin:100;closed[i]=physical[i]<=0;forceMin[i]=false;high[i]=false;}
 for(uint8_t i=0;i<2;++i){Printer::xyMotion.setMotorSteps(i,physical[i]);Printer::xyMotion.setNominalSteps(i,physical[i]);Printer::xyMotion.resetDirect(i);Printer::xyMotion.clearQueuedOvertravel(i);}
 auto& z=Printer::zMotion;z.configureTravel(100,10);z.setHomed(false);z.setHomed(true);z.setMotorReference(physical[Z_AXIS]);z.setNominalSteps(physical[Z_AXIS]);z.resetDirectPosition();z.clearScanAndCorrection();
 Printer::blockAll=Printer::printJob.blocked=false;Printer::homed=true;Printer::printJob.thermal=false;scanRunning=queueOccupied=false;driverEnabled=true;Calibration::scanSession.reset();z.setScanInProgress(false);pulses=0;
}
unsigned runStop(uint8_t axis,int32_t distance,uint8_t pack,unsigned inject,bool stoppable){
 reset(axis,distance>0?100:800);auto& m=PrintLine::direct;
 m.flags=FLAG_ABORT_AT_ENDSTOPS;m.dir=0;m.setMoveOfAxis(axis);if(distance>0)m.setPositiveDirectionForAxis(axis);
 for(auto& d:m.delta){d=0;}
 m.delta[axis]=m.stepsRemaining=std::abs(distance);m.primaryAxis=axis;m.stepProfile={};m.stepProfile.vStart=m.stepProfile.vEnd=m.stepProfile.vMax=5000;
 m.task=stoppable?DIRECT_PREPARED_STOPPABLE:DIRECT_PREPARED;
 if(axis==Z_AXIS)Printer::zMotion.requestDirectMove(distance);else Printer::xyMotion.requestDirect(axis,distance);
 const int32_t origin=motor(axis),oldNominal=nominal(axis),oldLedger=ledger(axis).steps();
 armed=requested=false;visits=0;injectAt=inject;StepExecutor::directionOwner=StepExecutor::DirectionOwner::None;
 {InterruptProtectedBlock lock;StepExecutor::performDirectMove();}
 assert(!pulses&&m.stepsRemaining==std::abs(distance));StepExecutor::stepsPerTimerCall=pack;armed=true;
 for(unsigned tick=0;PrintLine::directMoveActive();++tick){
  assert(tick<2000);{InterruptProtectedBlock lock;StepExecutor::performDirectMove();}
  assert(!high[0]&&!high[1]&&!high[2]);assert(m.stepsRemaining>=0);
  assert(motor(axis)==physical[axis]&&nominal(axis)==oldNominal);
  assert(StepExecutor::stepClocks[FOR_DIRECT].steps==pulses);
  assert(ledger(axis).steps()-oldLedger==motor(axis)-origin);
 }
 armed=false;assert(ledger(axis).steps()==ledger(axis).target());
 assert(pulses<=unsigned(std::abs(distance)));
 if(requested)assert(pulses-pulsesAt<=33);else assert(pulses==unsigned(std::abs(distance)));
 return visits;
}
unsigned mixed(uint8_t pack,unsigned inject,bool stoppable,int sign){
 reset(X_AXIS,sign>0?100:800);
 const int32_t distance[3]={sign*600,-sign*371,sign*129};
 for(uint8_t a=Y_AXIS;a<=Z_AXIS;++a){
  physical[a]=distance[a]>0?100:800;closed[a]=false;
  if(a==Y_AXIS){Printer::xyMotion.setMotorSteps(a,physical[a]);Printer::xyMotion.setNominalSteps(a,physical[a]);Printer::xyMotion.resetDirect(a);}
  else{Printer::zMotion.setMotorReference(physical[a]);Printer::zMotion.setNominalSteps(physical[a]);Printer::zMotion.resetDirectPosition();}
 }
 auto& m=PrintLine::direct;m.flags=FLAG_ABORT_AT_ENDSTOPS;m.dir=0;m.primaryAxis=X_AXIS;m.stepsRemaining=600;m.stepProfile={};m.stepProfile.vStart=m.stepProfile.vEnd=m.stepProfile.vMax=5000;m.delta[E_AXIS]=0;
 int32_t origins[3],nominals[3],oldLedgers[3];
 for(uint8_t a=X_AXIS;a<=Z_AXIS;++a){
  m.delta[a]=std::abs(distance[a]);m.setMoveOfAxis(a);if(distance[a]>0)m.setPositiveDirectionForAxis(a);
  if(a==Z_AXIS)Printer::zMotion.requestDirectMove(distance[a]);else Printer::xyMotion.requestDirect(a,distance[a]);
  origins[a]=motor(a);nominals[a]=nominal(a);oldLedgers[a]=ledger(a).steps();
 }
 m.task=stoppable?DIRECT_PREPARED_STOPPABLE:DIRECT_PREPARED;
 armed=requested=false;visits=0;injectAt=inject;StepExecutor::directionOwner=StepExecutor::DirectionOwner::None;
 {InterruptProtectedBlock lock;StepExecutor::performDirectMove();}
 assert(!pulses&&m.stepsRemaining==600);StepExecutor::stepsPerTimerCall=pack;armed=true;
 for(unsigned tick=0;PrintLine::directMoveActive();++tick){
  assert(tick<2000);{InterruptProtectedBlock lock;StepExecutor::performDirectMove();}
  assert(!high[0]&&!high[1]&&!high[2]&&m.stepsRemaining>=0);
  const int32_t n=std::abs(motor(X_AXIS)-origins[X_AXIS]);
  assert(StepExecutor::stepClocks[FOR_DIRECT].steps==unsigned(n));
  for(uint8_t a=X_AXIS;a<=Z_AXIS;++a){
   const int32_t delta=(n*std::abs(distance[a])+299)/600*(distance[a]>0?1:-1);
   assert(motor(a)==physical[a]&&nominal(a)==nominals[a]);
   assert(ledger(a).steps()-oldLedgers[a]==motor(a)-origins[a]&&motor(a)-origins[a]==delta);
  }
 }
 armed=false;for(uint8_t a=X_AXIS;a<=Z_AXIS;++a)assert(ledger(a).steps()==ledger(a).target());
 if(requested)assert(std::abs(physical[X_AXIS]-xAt)<=33);else assert(std::abs(physical[X_AXIS]-origins[X_AXIS])==600);
 return visits;
}
int main(){unsigned long cases=0;for(uint8_t a=0;a<=Z_AXIS;++a)for(int distance:{600,-600})for(uint8_t pack:{uint8_t(1),uint8_t(4),uint8_t(10),uint8_t(40)})for(bool stoppable:{false,true}){
 unsigned count=runStop(a,distance,pack,0,stoppable);for(unsigned at=1;at<=count;++at){runStop(a,distance,pack,at,stoppable);++cases;}
}std::printf("Actual direct pulse / ledger stop request interleavings: %lu cases PASS\n",cases);
 cases=0;for(int sign:{1,-1})for(uint8_t pack:{uint8_t(1),uint8_t(10),uint8_t(40)})for(bool stoppable:{false,true}){
 unsigned count=mixed(pack,0,stoppable,sign);for(unsigned at=1;at<=count;++at){mixed(pack,at,stoppable,sign);++cases;}
 }std::printf("Actual mixed XYZ pulse/ledger and clock interleavings: %lu cases PASS\n",cases);}

