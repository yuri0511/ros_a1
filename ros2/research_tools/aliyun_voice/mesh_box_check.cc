// Read-only FCL query. No ROS, Gazebo or actuator connections.
#include <fcl/fcl.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <vector>
#include <string>
struct Item {std::string group,name;std::shared_ptr<fcl::CollisionGeometryd> geom;fcl::Transform3d tf;};
int main(int argc,char**argv){
 if(argc!=2)return 2;std::ifstream in(argv[1]);std::map<int,std::vector<Item>> cases;
 std::map<std::string,std::shared_ptr<fcl::CollisionGeometryd>> cache;
 int k;std::string group,name,kind,path;double a,b,c,sx,sy,sz;
 while(in>>k>>group>>name>>kind>>a>>b>>c){
  fcl::Transform3d tf=fcl::Transform3d::Identity();for(int i=0;i<3;i++)for(int j=0;j<4;j++)in>>tf.matrix()(i,j);
  in>>std::quoted(path)>>sx>>sy>>sz;
  std::shared_ptr<fcl::CollisionGeometryd> geom;
  if(kind=="box")geom=std::make_shared<fcl::Boxd>(a,b,c);
  else if(kind=="cylinder")geom=std::make_shared<fcl::Cylinderd>(a,b);
  else if(kind=="sphere")geom=std::make_shared<fcl::Sphered>(a);
  else if(kind=="mesh"){
   std::string key=path+std::to_string(sx)+std::to_string(sy)+std::to_string(sz);
   if(cache.count(key))geom=cache[key];else{
    Assimp::Importer importer;const aiScene* scene=importer.ReadFile(path,aiProcess_Triangulate|aiProcess_PreTransformVertices);
    if(!scene){std::cerr<<"MESH_LOAD_FAILED "<<path<<"\n";return 3;}
    std::vector<fcl::Vector3d> vertices;std::vector<fcl::Triangle> triangles;
    for(unsigned n=0;n<scene->mNumMeshes;n++){
     auto mesh=scene->mMeshes[n];int offset=vertices.size();
     for(unsigned i=0;i<mesh->mNumVertices;i++){auto v=mesh->mVertices[i];vertices.emplace_back(v.x*sx,v.y*sy,v.z*sz);}
     for(unsigned i=0;i<mesh->mNumFaces;i++){auto f=mesh->mFaces[i];if(f.mNumIndices==3)triangles.emplace_back(offset+f.mIndices[0],offset+f.mIndices[1],offset+f.mIndices[2]);}
    }
    auto model=std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();model->beginModel();model->addSubModel(vertices,triangles);model->endModel();geom=model;cache[key]=geom;
   }
  }else {std::cerr<<"unsupported geometry";return 4;}
  cases[k].push_back({group,name,geom,tf});
 }
 std::cout<<"case,object1,object2,collision,penetration_depth_m,distance_m\n"<<std::setprecision(15);
 for(auto& entry:cases){auto& items=entry.second;
  for(size_t i=0;i<items.size();i++)for(size_t j=0;j<i;j++){
   auto& a=items[i];auto& b=items[j];bool pair=(a.group=="arm2"&&b.group=="bin")||(a.group=="bin"&&b.group=="arm2")||(a.group=="held"&&b.group=="bin")||(a.group=="bin"&&b.group=="held")||(a.group=="arm1"&&b.group=="arm2")||(a.group=="arm2"&&b.group=="arm1");if(!pair)continue;
   fcl::CollisionObjectd first(a.geom,a.tf),second(b.geom,b.tf);fcl::CollisionRequestd request;request.enable_contact=true;request.num_max_contacts=100;fcl::CollisionResultd result;fcl::collide(&first,&second,request,result);double depth=0.;std::vector<fcl::Contactd> contacts;result.getContacts(contacts);for(auto& x:contacts)depth=std::max(depth,x.penetration_depth);
   fcl::DistanceRequestd dr;fcl::DistanceResultd dd;double dist=fcl::distance(&first,&second,dr,dd);
   std::cout<<entry.first<<','<<a.name<<','<<b.name<<','<<result.isCollision()<<','<<depth<<','<<dist<<'\n';
  }
 }
}
