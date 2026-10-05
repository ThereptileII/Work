#include "ogr_s57.h"
#include "gdal/cpl_conv.h"
#include <iostream>
int main(int argc,char**argv){
 S57ClassRegistrar registrar;if(!registrar.LoadInfo(argv[1],false))return 2;
 OGRS57DataSource source;source.SetS57Registrar(&registrar);
 std::cerr<<"open="<<source.Open(argv[2],true)<<" layers="<<source.GetLayerCount()<<"\n";
 for(int i=0;i<source.GetModuleCount();++i){auto*layer=source.GetModule(i);layer->Rewind();
  while(auto*f=layer->ReadNextFeature()){
   auto*g=f->GetGeometryRef();if(g){OGREnvelope box;g->getEnvelope(&box);
    if(box.MinX < -122.33 && box.MaxX > -122.39 && box.MinY <47.62 && box.MaxY >47.58){
     const char*name=f->GetDefnRef()->GetName();
     if(std::string(name)!="SOUNDG"&&std::string(name)!="DEPCNT"&&std::string(name)!="DEPARE"){
      char*wkt=nullptr;g->exportToWkt(&wkt);std::cout<<name<<" FID="<<f->GetFID()<<" "<<wkt<<"\n";CPLFree(wkt);
      for(int j=0;j<f->GetFieldCount();++j)if(f->IsFieldSet(j))std::cout<<f->GetFieldDefnRef(j)->GetNameRef()<<"="<<f->GetFieldAsString(j)<<" ";std::cout<<"\n";
     }
    }
   } delete f;
  }
 }
}
