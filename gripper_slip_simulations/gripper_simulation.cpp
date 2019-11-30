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
    application.AddTypicalCamera(core::vector3df(0.5, 0.5, -1),
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

	//////////////////////////
	//  Left Outer Knuckle  //
	//////////////////////////
	auto leftOuterKnuckle = std::make_shared<ChBody>();
	leftOuterKnuckle->SetPos(ChVector<>(0.0306011444260539, 0, 0.0627920162695395));
	leftOuterKnuckle->SetMass(0.00684838849434396);
	leftOuterKnuckle->SetInertia(ChMatrix33<>(ChVector<>(2.66832029033166E-07, 1.3889233257419E-06, 1.26603336914415E-06),
		ChVector<>(1.66142314639824E-15, 1.45945633322873E-07, 2.82951161241588E-15)));
	//wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
	// NEED TO SET COLLISION

	auto leftOuterKnuckle_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftOuterKnuckle_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/outer_knuckle_fine.obj"));
	leftOuterKnuckle->AddAsset(leftOuterKnuckle_mesh);
	//leftOuterKnuckle->SetBodyFixed(true);
	mphysicalSystem.AddBody(leftOuterKnuckle);

	auto joint1 = std::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);  // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> joint_one_start(ChVector<>(0.0306011444260539, 0, 0.0627920162695395));

	joint1->Initialize(gripperBase,        // the 1st body to connect
		leftOuterKnuckle,           // the 2nd body to connect
		false,               // the two following frames are in absolute, not relative, coords.
		joint_one_start,   // the link reference attached to 1st body
		joint_one_start);  // the link reference attached to 2nd body
	//leftOuterKnuckle->SetPos_dt(ChVector<>(0.01, 0.01, 0.01));

	//mphysicalSystem.Add(joint1);

	//////////////////////////
	//  Left Inner Knuckle  //
	//////////////////////////
	auto leftInnerKnuckle = std::make_shared<ChBody>();
	leftInnerKnuckle->SetPos(ChVector<>(0.0127000000001501, 0, 0.0693074999999639));
	leftInnerKnuckle->SetMass(0.00684838849434396);
	leftInnerKnuckle->SetInertia(ChMatrix33<>(ChVector<>(2.66832029033166E-07, 1.3889233257419E-06, 1.26603336914415E-06),
		ChVector<>(1.66142314639824E-15, 1.45945633322873E-07, 2.82951161241588E-15)));
	//wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
	// NEED TO SET COLLISION

	auto leftInnerKnuckle_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftInnerKnuckle_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/inner_knuckle_fine.obj"));
	leftInnerKnuckle->AddAsset(leftInnerKnuckle_mesh);
	//leftInnerKnuckle->SetBodyFixed(true);
	mphysicalSystem.AddBody(leftInnerKnuckle);

	auto joint2 = std::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);  // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> joint_two_start(ChVector<>(0.0127000000001501, 0, 0.0693074999999639));

	joint2->Initialize(gripperBase,        // the 1st body to connect
		leftInnerKnuckle,           // the 2nd body to connect
		false,               // the two following frames are in absolute, not relative, coords.
		joint_two_start,   // the link reference attached to 1st body
		joint_two_start);  // the link reference attached to 2nd body
	mphysicalSystem.Add(joint2);

	//////////////////////////
	//  Left Outer Finger   //
	//////////////////////////
	auto leftOuterFinger = std::make_shared<ChBody>();
	leftOuterFinger->SetPos(ChVector<>(0.0316910442266543+ 0.0306011444260539, 0, -0.001933963757246050+.0627920162695395));
	leftOuterFinger->SetMass(0.00684838849434396);
	leftOuterFinger->SetInertia(ChMatrix33<>(ChVector<>(2.66832029033166E-07, 1.3889233257419E-06, 1.26603336914415E-06),
		ChVector<>(1.66142314639824E-15, 1.45945633322873E-07, 2.82951161241588E-15)));
	//wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
	// NEED TO SET COLLISION

	auto leftOuterFinger_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftOuterFinger_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/outer_finger_fine.obj"));
	leftOuterFinger->AddAsset(leftOuterFinger_mesh);
	leftOuterFinger->SetBodyFixed(true);
	mphysicalSystem.AddBody(leftOuterFinger);

	auto joint3 = std::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);  // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> joint_three_start(ChVector<>(0.0316910442266543 + 0.0306011444260539, 0, -0.001933963757246050 + .0627920162695395));

	joint3->Initialize(leftOuterKnuckle,        // the 1st body to connect
		leftOuterFinger,           // the 2nd body to connect
		false,               // the two following frames are in absolute, not relative, coords.
		joint_three_start,   // the link reference attached to 1st body
		joint_three_start);  // the link reference attached to 2nd body
	mphysicalSystem.Add(joint3);

	//////////////////////////
	//  Left Inner Finger   //
	//////////////////////////
	auto leftInnerFinger = std::make_shared<ChBody>();
	leftInnerFinger->SetPos(ChVector<>(0.034585310861294+0.0127000000001501, 0, 0.0454970193817975+ 0.0693074999999639));
	leftInnerFinger->SetMass(0.00684838849434396);
	leftInnerFinger->SetInertia(ChMatrix33<>(ChVector<>(2.66832029033166E-07, 1.3889233257419E-06, 1.26603336914415E-06),
		ChVector<>(1.66142314639824E-15, 1.45945633322873E-07, 2.82951161241588E-15)));
	//wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
	// NEED TO SET COLLISION

	auto leftInnerFinger_mesh = chrono_types::make_shared<ChObjShapeFile>();
	leftInnerFinger_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/inner_finger_fine.obj"));
	leftInnerFinger->AddAsset(leftInnerFinger_mesh);
	leftInnerFinger->SetBodyFixed(true);
	mphysicalSystem.AddBody(leftInnerFinger);

	auto joint4 = std::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);  // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> joint_four_start(ChVector<>(0.034585310861294 + 0.0127000000001501, 0, 0.0454970193817975 + 0.0693074999999639));

	joint4->Initialize(leftInnerKnuckle,        // the 1st body to connect
		leftInnerFinger,           // the 2nd body to connect
		false,               // the two following frames are in absolute, not relative, coords.
		joint_four_start,   // the link reference attached to 1st body
		joint_four_start);  // the link reference attached to 2nd body
	mphysicalSystem.Add(joint4);

	auto joint5 = std::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);  // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> joint_five_start(ChVector<>(0.034585310861294 + 0.0127000000001501, 0, 0.0454970193817975 + 0.0693074999999639));

	joint5->Initialize(leftInnerKnuckle,        // the 1st body to connect
		leftInnerFinger,           // the 2nd body to connect
		false,               // the two following frames are in absolute, not relative, coords.
		joint_five_start,   // the link reference attached to 1st body
		joint_five_start);  // the link reference attached to 2nd body
							//mphysicalSystem.Add(joint4);

	//////////////////////////
	//     Left Motor       //
	//////////////////////////

	auto left_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
	left_motor->Initialize(gripperBase, leftOuterKnuckle, ChFrame<>(ChVector<>(0.0306011444260539, 0, 0.0627920162695395)));
	left_motor->SetName("RotationalMotor");
	mphysicalSystem.AddLink(left_motor);
	auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
	left_motor->SetSpeedFunction(my_speed_function);

	//auto joint1 = std::make_shared<ChLinkLockRevolute>();
	//joint1->Initialize(gripper_marker_one, left_knuckle_marker);
	//mphysicalSystem.Add(joint1);

	//auto joint1 =
	//	std::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
	//ChFrame<> joint_one_start(ChVector<>(0, 2, 0));

	//joint1->Initialize(gripperBase,        // the 1st body to connect
	//	leftInnerKnuckle,           // the 2nd body to connect
	//	false,               // the two following frames are in absolute, not relative, coords.
	//	joint_one_start,   // the link reference attached to 1st body
	//	joint_one_start);  // the link reference attached to 2nd body

	//mphysicalSystem.Add(joint1);
	//leftInnerKnuckle->SetPos_dt(ChVector<>(0.01, 0, 0));

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
