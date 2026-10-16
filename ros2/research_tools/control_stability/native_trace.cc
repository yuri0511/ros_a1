// Read-only ECM observations. No components, commands, forces or entities are mutated.
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/plugin/Register.hh>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
namespace research {
class JointTrace final : public gz::sim::System, public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
 std::ofstream out; std::map<gz::sim::Entity,std::string> joints;
 std::map<gz::sim::Entity,std::pair<double,double>> pre; std::set<std::string> wanted; uint64_t rows=0; double flushed=0;
 public:
 void Configure(const gz::sim::Entity&, const std::shared_ptr<const sdf::Element>&,
                gz::sim::EntityComponentManager&, gz::sim::EventManager&) override {
  const char *path=std::getenv("CONTROL_NATIVE_CSV"); if(!path)return;
  for(auto prefix:{"","arm2_"})for(auto n:{"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"})wanted.insert(std::string(prefix)+n);
  out.open(path);out<<std::setprecision(17)<<"iteration,sim_sec,paused,entity,joint,q,v,velocity_cmd,pre_q,velocity_cmd_pre\n";out.flush();
 }
 void PreUpdate(const gz::sim::UpdateInfo &,gz::sim::EntityComponentManager &ecm) override {
  // Priority 100: after default-priority command writers, before Update physics.
  // The non-const API is required by ISystemPreUpdate; only const reads are used.
  const auto &read=ecm;
  for(const auto &[entity,name]:joints){
   const auto *q=read.Component<gz::sim::components::JointPosition>(entity);
   const auto *c=read.Component<gz::sim::components::JointVelocityCmd>(entity);
   if(q&&c&&!q->Data().empty()&&!c->Data().empty())pre[entity]={q->Data()[0],c->Data()[0]};
  }
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
  if(!out||info.paused||rows>=5000000)return;
  double sim=std::chrono::duration<double>(info.simTime).count(); if(sim>600)return;
  if(joints.size()<12)ecm.Each<gz::sim::components::Joint,gz::sim::components::Name>([&](const gz::sim::Entity&e,const auto*,const auto*n){if(wanted.count(n->Data()))joints[e]=n->Data();return true;});
  for(const auto &[entity,name]:joints){
   const auto *q=ecm.Component<gz::sim::components::JointPosition>(entity);
   const auto *v=ecm.Component<gz::sim::components::JointVelocity>(entity);
   const auto *c=ecm.Component<gz::sim::components::JointVelocityCmd>(entity);
   if(!q||!v||!c||q->Data().empty()||v->Data().empty()||c->Data().empty())continue;
   out<<info.iterations<<','<<sim<<",0,"<<entity<<','<<name<<','<<q->Data()[0]<<','<<v->Data()[0]<<','<<c->Data()[0]<<','<<(pre.count(entity)?pre.at(entity).first:0.)<<','<<(pre.count(entity)?pre.at(entity).second:0.)<<'\n';++rows;
  }
  if(sim-flushed>=.5){out.flush();flushed=sim;}
 }
};
}
GZ_ADD_PLUGIN(research::JointTrace,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
