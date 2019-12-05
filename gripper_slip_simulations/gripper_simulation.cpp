// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//  Modified by Mike Hagenow 11-28-19
//  Used to create the Robotiq gripper geometry that is used in Simulation
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create a Chrono physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Gripper Kinematic Simulation", core::dimension2d<u32>(800, 600),
                         false);  // screen dimensions

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    //application.AddTypicalCamera(core::vector3df(0.5, 0.5, -1),
     //                            core::vector3df(0, 0.1, 0));  // to change the position of camera

	application.AddTypicalCamera(core::vector3df(0, 0.2, 0),
		core::vector3df(0, 0.1, 0));  // to change the position of camera
   
	// Floor for simulation perspective
	auto floorBody = std::make_shared<ChBodyEasyBox>(0.5, 0.01, 0.5,  // x, y, z dimensions
		3000,       // density
		false,      // no contact geometry
		true        // enable visualization geometry
		);
	floorBody->SetPos(ChVector<>(0, -0.2, 0));
	floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

	//////////////////////////////////////////////////
	///       DEFINES THE GRIPPER GEOMETRY         ///
	//////////////////////////////////////////////////

	//======================================================================
	// The gripper model comes from the Robotiq Two-Finger 85 Model.
	// This mechanism consists of 9 links supported by 8 joints
	// The geometry is defined below and was derived in terms of geometric
	// primitives based on the following:
	// https://github.com/a-price/robotiq_arg85_description/blob/master/robots/robotiq_arg85_description.URDF
	// As far as constraints, proximal links are defined at the same time as the geometry
	//======================================================================

	/////////////////////
	//  Gripper Base   //
	/////////////////////
	auto gripperBase = std::make_shared<ChBody>();
	gripperBase->SetPos(ChVector<>(0.0, 0.0, 0.0));
	gripperBase->SetMass(0.30915);
	gripperBase->SetInertia(ChMatrix33<>(ChVector<>(0.00028972, 0.00030737, 0.00019914), ChVector<>(-5.7879E-10, -1.8543E-06, 1.682E-12)));
	//wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
	// NEED TO SET COLLISION
	
	auto gripperBase_mesh = chrono_types::make_shared<ChObjShapeFile>();
	gripperBase_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/robotiq_85_base_link_fine.obj"));
	gripperBase->AddAsset(gripperBase_mesh);
	gripperBase->SetBodyFixed(true);
	mphysicalSystem.AddBody(gripperBase);
	auto marker_gripperBase = std::make_shared<ChMarker>();
	marker_gripperBase->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0127000000001501, 0, 0.0693074999999639), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	gripperBase->AddMarker(marker_gripperBase);

	auto marker_gripperBaseRight = std::make_shared<ChMarker>();
	marker_gripperBaseRight->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(-0.0127000000001501, 0, 0.0693074999999639), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
	gripperBase->AddMarker(marker_gripperBaseRight);


	auto point = std::make_shared<ChBodyEasySphere>(0.005,1000,true,true);
	point->SetPos(ChVector<>(-0.065, 0.01, 0.109));
	//point->SetPos(ChVector<>(0, 0, 0));
	point->SetBodyFixed(true);
	mphysicalSystem.AddBody(point);


	//////////////////////////
	//  Left Outer Knuckle  //
	//////////////////////////
	auto leftOuterKnuckle = std::make_shared<ChBody>();
	leftOuterKnuckle->SetPos(ChVector<>(0.0306011444260539, 0, 0.0627920162695395));
	leftOuterKnuckle->SetBodyFixed(true);
	leftOuterKnuckle->SetMass(0.00684838849434396);
	leftOuterKnuckle->SetInertia(ChMatrix33<>(ChVector<>(2.66832029033166E-07, 1.3889233257419E-06, 1.26603336914415E-06),
		ChVector<>(1.66142314639824E-15, 1.45945633322873E-07, 2.82951161241588E-15)));

	auto leftOuterKnuckle_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftOuterKnuckle_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/outer_knuckle_fine.obj"));
	leftOuterKnuckle->AddAsset(leftOuterKnuckle_mesh);
	mphysicalSystem.AddBody(leftOuterKnuckle);
	
	auto marker_leftOuterKnuckle = std::make_shared<ChMarker>();
	marker_leftOuterKnuckle->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0316910442266543, 0, -0.001933963757246050), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftOuterKnuckle->AddMarker(marker_leftOuterKnuckle);
	
	//////////////////////////
	//  Right Outer Knuckle  //
	//////////////////////////
	auto rightOuterKnuckle = std::make_shared<ChBody>();
	rightOuterKnuckle->SetPos(ChVector<>(-0.0306011444260539, 0, 0.0627920162695395));
	rightOuterKnuckle->SetRot(ChQuaternion<>(0, 0, 0, 1));
	//rightOuterKnuckle->SetBodyFixed(true);
	rightOuterKnuckle->SetMass(0.00684838849434396);
	rightOuterKnuckle->SetInertia(ChMatrix33<>(ChVector<>(2.66832029033166E-07, 1.3889233257419E-06, 1.26603336914415E-06),
		ChVector<>(1.66142314639824E-15, 1.45945633322873E-07, 2.82951161241588E-15)));

	auto rightOuterKnuckle_mesh = chrono_types::make_shared<ChObjShapeFile>();
	rightOuterKnuckle_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/outer_knuckle_fine.obj"));
	rightOuterKnuckle->AddAsset(rightOuterKnuckle_mesh);
	mphysicalSystem.AddBody(rightOuterKnuckle);

	auto marker_rightOuterKnuckle = std::make_shared<ChMarker>();
	marker_rightOuterKnuckle->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0316910442266543, 0, -0.001933963757246050), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	rightOuterKnuckle->AddMarker(marker_rightOuterKnuckle);

	//////////////////////////
	//  Left Inner Knuckle  //
	//////////////////////////
	auto leftInnerKnuckle = std::make_shared<ChBody>();
	leftInnerKnuckle->SetPos(ChVector<>(0.0127000000001501, 0, 0.0693074999999639));
	leftInnerKnuckle->SetMass(0.0110930853895903);
	leftInnerKnuckle->SetInertia(ChMatrix33<>(ChVector<>(4.23392770691541E-06, 3.96548790524392E-06, 3.24068002883007E-06),
		ChVector<>(5.748978936968E-15, 1.78855677119788E-06, 1.05464666369669E-14)));
	

	auto leftInnerKnuckle_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftInnerKnuckle_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/inner_knuckle_fine.obj"));
	leftInnerKnuckle->AddAsset(leftInnerKnuckle_mesh);
	mphysicalSystem.AddBody(leftInnerKnuckle);

	auto marker_leftInnerKnuckle = std::make_shared<ChMarker>();
	marker_leftInnerKnuckle->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftInnerKnuckle->AddMarker(marker_leftInnerKnuckle);

	auto joint1 = std::make_shared<ChLinkLockRevolute>();
	joint1->Initialize(marker_gripperBase, marker_leftInnerKnuckle);
	mphysicalSystem.Add(joint1);

	auto marker_leftInnerKnuckle_finger = std::make_shared<ChMarker>();
	marker_leftInnerKnuckle_finger->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.034585310861294, 0, 0.0454970193817975), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftInnerKnuckle->AddMarker(marker_leftInnerKnuckle_finger);

	//////////////////////////
	//  Right Inner Knuckle  //
	//////////////////////////
	auto rightInnerKnuckle = std::make_shared<ChBody>();
	rightInnerKnuckle->SetPos(ChVector<>(-0.0127000000001501, 0, 0.0693074999999639));
	rightInnerKnuckle->SetRot(ChQuaternion<>(0, 0, 0, 1));
	//rightInnerKnuckle->SetBodyFixed(true);
	rightInnerKnuckle->SetMass(0.0110930853895903);
	rightInnerKnuckle->SetInertia(ChMatrix33<>(ChVector<>(4.23392770691541E-06, 3.96548790524392E-06, 3.24068002883007E-06),
		ChVector<>(5.748978936968E-15, 1.78855677119788E-06, 1.05464666369669E-14)));


	auto rightInnerKnuckle_mesh = chrono_types::make_shared<ChObjShapeFile>();
	rightInnerKnuckle_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/inner_knuckle_fine.obj"));
	rightInnerKnuckle->AddAsset(rightInnerKnuckle_mesh);
	mphysicalSystem.AddBody(rightInnerKnuckle);

	auto marker_rightInnerKnuckle = std::make_shared<ChMarker>();
	marker_rightInnerKnuckle->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	rightInnerKnuckle->AddMarker(marker_rightInnerKnuckle);

	auto joint1right = std::make_shared<ChLinkLockRevolute>();
	joint1right->Initialize(marker_gripperBaseRight, marker_rightInnerKnuckle);
	mphysicalSystem.Add(joint1right);

	auto marker_rightInnerKnuckle_finger = std::make_shared<ChMarker>();
	marker_rightInnerKnuckle_finger->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.034585310861294, 0, 0.0454970193817975), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	rightInnerKnuckle->AddMarker(marker_rightInnerKnuckle_finger);

	//////////////////////////
	//  Left Outer Finger   //
	//////////////////////////
	auto leftOuterFinger = std::make_shared<ChBody>();
	leftOuterFinger->SetPos(ChVector<>(0.0316910442266543+ 0.0306011444260539, 0, -0.001933963757246050+.0627920162695395));
	leftOuterFinger->SetMass(0.0273093985570947);
	leftOuterFinger->SetInertia(ChMatrix33<>(ChVector<>(8.51629628283022E-06, 6.9133328065108E-06, 2.25006832221981E-06),
		ChVector<>(2.58174336207405E-19, 3.83829504344079E-07, -7.58589926143789E-19)));
	

	auto leftOuterFinger_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftOuterFinger_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/outer_finger_fine.obj"));
	leftOuterFinger->AddAsset(leftOuterFinger_mesh);
	//leftOuterFinger->SetBodyFixed(true);
	mphysicalSystem.AddBody(leftOuterFinger);

	auto marker_leftOuterFinger = std::make_shared<ChMarker>();
	marker_leftOuterFinger->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftOuterFinger->AddMarker(marker_leftOuterFinger);

	auto joint2 = std::make_shared<ChLinkLockRevolute>();
	joint2->Initialize(marker_leftOuterKnuckle, marker_leftOuterFinger);
	mphysicalSystem.Add(joint2);

	auto marker_leftOuterFinger_Assumed = std::make_shared<ChMarker>();
	marker_leftOuterFinger_Assumed->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0.0431), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftOuterFinger->AddMarker(marker_leftOuterFinger_Assumed);

	//////////////////////////
	//  Right Outer Finger   //
	//////////////////////////
	auto rightOuterFinger = std::make_shared<ChBody>();
	rightOuterFinger->SetPos(ChVector<>(-0.0316910442266543 - 0.0306011444260539, 0, -0.001933963757246050 + .0627920162695395));
	rightOuterFinger->SetRot(ChQuaternion<>(0, 0, 0, 1));
	rightOuterFinger->SetMass(0.0273093985570947);
	rightOuterFinger->SetInertia(ChMatrix33<>(ChVector<>(8.51629628283022E-06, 6.9133328065108E-06, 2.25006832221981E-06),
		ChVector<>(2.58174336207405E-19, 3.83829504344079E-07, -7.58589926143789E-19)));


	auto rightOuterFinger_mesh = chrono_types::make_shared<ChObjShapeFile>();
	rightOuterFinger_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/outer_finger_fine.obj"));
	rightOuterFinger->AddAsset(rightOuterFinger_mesh);
	//rightOuterFinger->SetBodyFixed(true);
	mphysicalSystem.AddBody(rightOuterFinger);

	auto marker_rightOuterFinger = std::make_shared<ChMarker>();
	marker_rightOuterFinger->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	rightOuterFinger->AddMarker(marker_rightOuterFinger);

	auto joint2right = std::make_shared<ChLinkLockRevolute>();
	joint2right->Initialize(marker_rightOuterKnuckle, marker_rightOuterFinger);
	mphysicalSystem.Add(joint2right);

	auto marker_rightOuterFinger_Assumed = std::make_shared<ChMarker>();
	marker_rightOuterFinger_Assumed->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0648-.06229, 0, 0.105-0.06089), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	//marker_rightOuterFinger_Assumed->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0.104-0.06089), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	rightOuterFinger->AddMarker(marker_rightOuterFinger_Assumed);

	//////////////////////////
	//  Left Inner Finger   //
	//////////////////////////
	auto leftInnerFinger = std::make_shared<ChBody>();
	leftInnerFinger->SetPos(ChVector<>(0.034585310861294+0.0127000000001501, 0, 0.0454970193817975+ 0.0693074999999639));
	leftInnerFinger->SetMass(0.00724255346165745);
	leftInnerFinger->SetInertia(ChMatrix33<>(ChVector<>(1.47824274053603E-06, 1.70064480838395E-06, 4.77151336838364E-07),
		ChVector<>(-3.94884463570303E-19, -3.45268847648622E-07, 4.77151336838364E-07)));
	
	auto leftInnerFinger_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftInnerFinger_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/inner_finger_fine.obj"));
	leftInnerFinger->AddAsset(leftInnerFinger_mesh);
	//leftInnerFinger->SetBodyFixed(true);
	mphysicalSystem.AddBody(leftInnerFinger);

	auto marker_leftInnerFinger = std::make_shared<ChMarker>();
	marker_leftInnerFinger->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftInnerFinger->AddMarker(marker_leftInnerFinger);

	auto joint3 = std::make_shared<ChLinkLockRevolute>();
	joint3->Initialize(marker_leftInnerKnuckle_finger, marker_leftInnerFinger);
	mphysicalSystem.Add(joint3);

	////// 

	auto marker_leftInnerFinger_Assumed = std::make_shared<ChMarker>();
	marker_leftInnerFinger_Assumed->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.01472, 0, -0.0108), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftInnerFinger->AddMarker(marker_leftInnerFinger_Assumed);

	auto joint4 = std::make_shared<ChLinkLockRevolute>();
	joint4->Initialize(marker_leftOuterFinger_Assumed, marker_leftInnerFinger_Assumed);
	mphysicalSystem.Add(joint4); 

	//////////////////////////
	//  Right Inner Finger   //
	//////////////////////////
	auto rightInnerFinger = std::make_shared<ChBody>();
	rightInnerFinger->SetPos(ChVector<>(-0.034585310861294 - 0.0127000000001501, 0, 0.0454970193817975 + 0.0693074999999639));
	rightInnerFinger->SetRot(ChQuaternion<>(0, 0, 0, 1));
	rightInnerFinger->SetMass(0.00724255346165745);
	rightInnerFinger->SetInertia(ChMatrix33<>(ChVector<>(1.47824274053603E-06, 1.70064480838395E-06, 4.77151336838364E-07),
		ChVector<>(-3.94884463570303E-19, -3.45268847648622E-07, 4.77151336838364E-07)));

	auto rightInnerFinger_mesh = chrono_types::make_shared<ChObjShapeFile>();
	rightInnerFinger_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/inner_finger_fine.obj"));
	rightInnerFinger->AddAsset(rightInnerFinger_mesh);
	//rightInnerFinger->SetBodyFixed(true);
	mphysicalSystem.AddBody(rightInnerFinger);

	auto marker_rightInnerFinger = std::make_shared<ChMarker>();
	marker_rightInnerFinger->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	rightInnerFinger->AddMarker(marker_rightInnerFinger);

	auto joint3right = std::make_shared<ChLinkLockRevolute>();
	joint3right->Initialize(marker_rightInnerKnuckle_finger, marker_rightInnerFinger);
	mphysicalSystem.Add(joint3right);

	////// 

	auto marker_rightInnerFinger_Assumed = std::make_shared<ChMarker>();
	marker_rightInnerFinger_Assumed->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0648-0.04728, 0, 0.105-0.11479), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	rightInnerFinger->AddMarker(marker_rightInnerFinger_Assumed);

	auto joint4right = std::make_shared<ChLinkLockRevolute>();
	joint4right->Initialize(marker_rightOuterFinger_Assumed, marker_rightInnerFinger_Assumed);
	mphysicalSystem.Add(joint4right);

	//////////////////////////
	//     Left Motor       //
	//////////////////////////

	auto left_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
	left_motor->Initialize(gripperBase, leftOuterKnuckle, ChFrame<>(ChVector<>(0.0306011444260539, 0, 0.0627920162695395), Q_from_AngAxis(-CH_C_PI_2, VECT_X))); //
	left_motor->SetName("RotationalMotor");
	//left_motor->SetMotorFunction(ChFunction<>())
	
	mphysicalSystem.AddLink(left_motor);
	auto left_speed_function = chrono_types::make_shared<ChFunction_Const>(0.1);  // speed w=3.145 rad/sec CH_C_PI/2
	left_motor->SetSpeedFunction(left_speed_function);

	//////////////////////////
	//     Right Motor       //
	//////////////////////////

	auto right_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
	right_motor->Initialize(gripperBase, rightOuterKnuckle, ChFrame<>(ChVector<>(-0.0306011444260539, 0, 0.0627920162695395), Q_from_AngAxis(-CH_C_PI_2, VECT_X))); //
	right_motor->SetName("RotationalMotor");
	//left_motor->SetMotorFunction(ChFunction<>())

	mphysicalSystem.AddLink(right_motor);
	auto right_speed_function = chrono_types::make_shared<ChFunction_Const>(-0.1);  // speed w=3.145 rad/sec CH_C_PI/2
	right_motor->SetSpeedFunction(right_speed_function);



    // Floor Color for Irrichlet Simulation
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.1f, 0.1f, 0.1f));
    floorBody->AddAsset(color);

    // Optionally, attach a texture to the pendulum, for better visualization
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));  // texture in ../data
    //pendulumBody->AddAsset(texture);


	// Gripper Colors
	//auto gripper_gray = std::make_shared<ChColorAsset>();
	//color->SetColor(ChColor(0.8f, 0.8f, 0.8f));
	//gripperBase->AddAsset(texture);
	//leftInnerKnuckle->AddAsset(texture);
	point->AddAsset(texture);

    //======================================================================
	//           Creates the Simulation through Irrich                    //
	//======================================================================

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
    application.SetTimestep(0.005);
    application.SetTryRealtime(true);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        application.DoStep();

        application.EndScene();
    }

    return 0;
}
