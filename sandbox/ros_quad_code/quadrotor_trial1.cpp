#include <iostream>
#include "vicon_pos.h"
using namespace ViconDataStreamSDK::CPP;
//using namespace ViconDataStreamSDK::CPP::vicon_pos;
using namespace std;


int main( int argc, char* argv[] )
{
float coord[3], Eulerxyz[3];
float index;
vicon_pos* c = new vicon_pos("Xbar","Unlabeled1","");

//ViconDataStreamSDK::Client* my_client;
//ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation temp_segment;
//ViconDataStreamSDK::CPP::Output_GetMarkerGlobalTranslation temp_marker;
//ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationEulerXYZ temp_rpy;

for(;;)
{
c->update();
//index = c->find_sub("Quad1");
c->get_subject_coord("Quad1",coord[0], coord[1], coord[2]);
c->get_subject_euler_xyz("Quad1",Eulerxyz[0],Eulerxyz[1],Eulerxyz[2]);

cout <<"coord" << coord[0] << "\t"<<coord[1] <<"\t"<< coord[2]<< endl;
cout << "Eulerxyz" << Eulerxyz[0] <<"\t"<<Eulerxyz[1] <<"\t"<< Eulerxyz[2] << endl;
//cout <<"subindex" << index << endl;


} //end of for


} //end of main

