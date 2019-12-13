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
// Modified by mhagenow
// Playground for testing effects of friction and collision parameters
// One half of the gripper interacts with a block on a table
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"


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
    
    // Create a Chrono physical system - complementarity
    ChSystemNSC mphysicalSystem;
	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0005);

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Gripper Complementarity Friction Testing Pushing", core::dimension2d<u32>(800, 600),
                         false);  // screen dimensions

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
   
	application.AddTypicalCamera(core::vector3df(0.05, 0.1, 0.25),
		core::vector3df(0, 0, 0.05));  // to change the position of camera
   
	// Floor for simulation perspective
	auto floorBody = std::make_shared<ChBodyEasyBox>(0.5, 0.01, 0.5,  // x, y, z dimensions
		3000,       // density
		true,      // no contact geometry
		true        // enable visualization geometry
		);
	floorBody->SetPos(ChVector<>(0, -0.04, 0.0));
	floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

	//////////////////////////////////////////////////
	///       DEFINES THE GRIPPER GEOMETRY         ///
	//////////////////////////////////////////////////

	//======================================================================
	// The gripper model comes from the Robotiq Two-Finger 85 Model.
	// This mechanism consists of 9 links supported by 8 joints
	// For this case, only half of the gripper is modeled to test the interaction


	// The geometry is defined below and was derived in terms of geometric
	// primitives based on the following:
	// https://github.com/a-price/robotiq_arg85_description/blob/master/robots/robotiq_arg85_description.URDF
	// As far as constraints, proximal links are defined at the same time as the geometry
	//======================================================================

	/////////////////////
	//  Gripper Base   //
	/////////////////////
	auto gripperBase = std::make_shared<ChBody>();
	gripperBase->SetPos(ChVector<>(0.0, 0.0,0));
	gripperBase->SetMass(0.30915);
	gripperBase->SetInertia(ChMatrix33<>(ChVector<>(0.00028972, 0.00030737, 0.00019914), ChVector<>(-5.7879E-10, -1.8543E-06, 1.682E-12)));
	//wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
	// NEED TO SET COLLISION
	
	auto gripperBase_mesh = chrono_types::make_shared<ChObjShapeFile>();
	gripperBase_mesh->SetFilename(GetChronoDataFile("../../gripper_geometry/robotiq_85_base_link_fine.obj"));
	gripperBase->AddAsset(gripperBase_mesh);
	gripperBase->SetBodyFixed(true);
	mphysicalSystem.AddBody(gripperBase);

	auto marker_gripperBaseLeft = std::make_shared<ChMarker>();
	marker_gripperBaseLeft->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0127000000001501, 0, 0.0693074999999639), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	gripperBase->AddMarker(marker_gripperBaseLeft);

	auto marker_gripperBaseRight = std::make_shared<ChMarker>();
	marker_gripperBaseRight->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(-0.0127000000001501, 0, 0.0693074999999639), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
	gripperBase->AddMarker(marker_gripperBaseRight);


	//////////////////////////
	//  Left Outer Knuckle  //
	//////////////////////////
	auto leftOuterKnuckle = std::make_shared<ChBody>();
	leftOuterKnuckle->SetPos(ChVector<>(0.0306011444260539, 0, 0.0627920162695395));
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

	leftOuterKnuckle->GetCollisionModel()->ClearModel();
	leftOuterKnuckle->GetCollisionModel()->AddBox(0.014, 0.02, 0.004, ChVector<>(0.0306011444260539 + 0.019, 0, 0.0627920162695395), ChQuaternion<>(1, 0, 0, 0));
	leftOuterKnuckle->GetCollisionModel()->BuildModel();
	leftOuterKnuckle->SetCollide(true);
	leftOuterKnuckle->GetMaterialSurfaceNSC()->SetFriction(0.2f);


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

	auto joint1left = std::make_shared<ChLinkLockRevolute>();
	joint1left->Initialize(marker_gripperBaseLeft, marker_leftInnerKnuckle);
	mphysicalSystem.Add(joint1left);

	auto marker_leftInnerKnuckle_finger = std::make_shared<ChMarker>();
	marker_leftInnerKnuckle_finger->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.034585310861294, 0, 0.0454970193817975), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftInnerKnuckle->AddMarker(marker_leftInnerKnuckle_finger);

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

	auto joint2left = std::make_shared<ChLinkLockRevolute>();
	joint2left->Initialize(marker_leftOuterKnuckle, marker_leftOuterFinger);
	mphysicalSystem.Add(joint2left);

	auto marker_leftOuterFinger_Assumed = std::make_shared<ChMarker>();
	marker_leftOuterFinger_Assumed->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0652 - .06229, 0, 0.1086 - 0.06089), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftOuterFinger->AddMarker(marker_leftOuterFinger_Assumed);

	leftOuterFinger->GetCollisionModel()->ClearModel();
	leftOuterFinger->GetCollisionModel()->AddBox(0.008, 0.02, 0.035, ChVector<>(0.034585310861294 + 0.0127000000001501 + 0.018, 0.00, 0.0454970193817975 + 0.0693074999999639 - 0.03), ChQuaternion<>(1, 0, 0, 0));
	leftOuterFinger->GetCollisionModel()->BuildModel();
	leftOuterFinger->SetCollide(true);
	leftOuterFinger->GetMaterialSurfaceNSC()->SetFriction(0.2f);

	

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

	auto joint3left = std::make_shared<ChLinkLockRevolute>();
	joint3left->Initialize(marker_leftInnerKnuckle_finger, marker_leftInnerFinger);
	mphysicalSystem.Add(joint3left);

	//////// 

	auto marker_leftInnerFinger_Assumed = std::make_shared<ChMarker>();
	marker_leftInnerFinger_Assumed->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(0.0652 - 0.04728, 0, 0.1086 - 0.11479), Q_from_AngAxis(-CH_C_PI_2, VECT_X)));
	leftInnerFinger->AddMarker(marker_leftInnerFinger_Assumed);

	auto joint4left = std::make_shared<ChLinkLockRevolute>();
	joint4left->Initialize(marker_leftOuterFinger_Assumed, marker_leftInnerFinger_Assumed);
	mphysicalSystem.Add(joint4left); 

	// Create a box to use for positioning of collision information
	//auto point = std::make_shared<ChBodyEasyBox>(0.008, 0.02, 0.035, 10000, false, true);
	//point->SetPos(ChVector<>(0.034585310861294 + 0.0127000000001501-0.003, 0, 0.0454970193817975 + 0.0693074999999639 +0.025));
	//point->SetBodyFixed(true);
	//mphysicalSystem.AddBody(point);

	// Collision Information for the Gripper
	// information provided in http://api.projectchrono.org/collision_shapes.html
	leftInnerFinger->GetCollisionModel()->ClearModel();
	leftInnerFinger->GetCollisionModel()->AddBox(0.008, 0.02, 0.035, ChVector<>(-0.000, 0, 0.025), ChQuaternion<>(1, 0, 0, 0));
	leftInnerFinger->GetCollisionModel()->BuildModel();
	leftInnerFinger->SetCollide(true);
	leftInnerFinger->GetMaterialSurfaceNSC()->SetFriction(0.2f);
	
	
	//////////////////////////
	//   BLOCK TO PUSH      //
	//////////////////////////

	auto pushing_block_mat = std::make_shared<ChMaterialSurfaceNSC>();
	pushing_block_mat->SetFriction(0.5f);
	pushing_block_mat->SetRollingFriction(0.001f);


	auto pushing_block = std::make_shared<ChBodyEasyBox>(0.05, 0.06, 0.05, 80, true, true);
	pushing_block->SetPos(ChVector<>(0, 0, 0.0454970193817975 + 0.0693074999999639 + 0.020));
	pushing_block->SetBodyFixed(false);
	pushing_block->SetMaterialSurface(pushing_block_mat);
	mphysicalSystem.AddBody(pushing_block);





	//////////////////////////
	//     DRIVING CODE     //
	//////////////////////////

	auto slip_grip_torque = std::make_shared<ChFunction_Const>(0.01); // 0.5 Nm (soft grip) -> Gripper can provide maximum of 3 Nm

	///////////////////////////////////
	//     Left Motor - Torque       //
	///////////////////////////////////

	auto left_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
	left_motor->Initialize(gripperBase, leftOuterKnuckle, ChFrame<>(ChVector<>(0.0306011444260539, 0, 0.0627920162695395), Q_from_AngAxis(-CH_C_PI_2, VECT_X))); //
	left_motor->SetName("RotationalMotor");
	left_motor->SetMotorFunction(slip_grip_torque);

	mphysicalSystem.AddLink(left_motor);
	
	
	/////////////////////////////////////////////
	// Lock the fingers - Uncomment if desired //
	//leftOuterKnuckle->SetBodyFixed(true);
	/////////////////////////////////////////////


	/////////////////////////////////
	//     Simulation Settings     //
	/////////////////////////////////

    // Floor Color for Irrichlet Simulation
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.3f, 0.2f));
    floorBody->AddAsset(color);

	// Gripper Colors
	auto gripper_gray = std::make_shared<ChColorAsset>();
	gripper_gray->SetColor(ChColor(0.2f, 0.2f, 0.2f));
	auto gripper_black = std::make_shared<ChColorAsset>();
	gripper_black->SetColor(ChColor(0.1f, 0.1f, 0.1f));
	gripperBase->AddAsset(gripper_gray);
	leftInnerKnuckle->AddAsset(gripper_gray);
	leftOuterKnuckle->AddAsset(gripper_gray);
	leftOuterFinger->AddAsset(gripper_gray);
	leftInnerFinger->AddAsset(gripper_black);

	// Add Color to Pushing Block - Green
	auto block_green = std::make_shared<ChColorAsset>();
	block_green->SetColor(ChColor(0.0f, 0.6078f, 0.0f));
	pushing_block->AddAsset(block_green);
	

    //======================================================================
	//           Creates the Simulation through Irrich                    //
	//======================================================================

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
    application.SetTimestep(0.00002);
    application.SetTryRealtime(true);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

	long iter = 0;
	while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
		// Runs for a limited period of time then stops doing integration
		if (iter < 320E2) {
			application.DoStep();
		}
        application.EndScene();
		iter++;
    }

	

    return 0;
}
