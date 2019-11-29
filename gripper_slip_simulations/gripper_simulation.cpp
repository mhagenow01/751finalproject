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
    application.AddTypicalCamera(core::vector3df(2, 2, -5),
                                 core::vector3df(0, 1, 0));  // to change the position of camera
   
	// Floor for simulation perspective
	auto floorBody = std::make_shared<ChBodyEasyBox>(5, 0.1, 5,  // x, y, z dimensions
		3000,       // density
		false,      // no contact geometry
		true        // enable visualization geometry
		);
	floorBody->SetPos(ChVector<>(0, 0, 0));
	floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

	//////////////////////////////////////////////////
	///       DEFINES THE GRIPPER GEOMETRY         ///
	//////////////////////////////////////////////////

	//======================================================================
	// The gripper model comes from the Robotiq Two-Finger 85 Model.
	// This mechanism consists of 9 links supported by 8 joints
	// The geometry is defined below and was taken derived based on the following:
	// https://github.com/a-price/robotiq_arg85_description/blob/master/robots/robotiq_arg85_description.URDF
	//======================================================================

	auto gripperBase = std::make_shared<ChBodyEasyBox>(0.4, 0.3, 0.3,3000,false, true);
	gripperBase->SetPos(ChVector<>(0, 2, -2));
	gripperBase->SetBodyFixed(true);
	auto gripper_marker_one = std::make_shared<ChMarker>();
	gripperBase->AddMarker(gripper_marker_one);

	auto leftInnerKnuckle = std::make_shared<ChBodyEasyBox>(0.1, 0.8, 0.1, 3000, false, true);
	leftInnerKnuckle->SetPos(ChVector<>(0, 2.4, -2));
	auto left_knuckle_marker = std::make_shared<ChMarker>();
	leftInnerKnuckle->AddMarker(left_knuckle_marker);
	//leftInnerKnuckle->SetBodyFixed(true);

	mphysicalSystem.Add(gripperBase);
	mphysicalSystem.Add(leftInnerKnuckle);

	//auto joint1 = std::make_shared<ChLinkLockRevolute>();
	//joint1->Initialize(gripper_marker_one, left_knuckle_marker);
	//mphysicalSystem.Add(joint1);

	auto joint1 =
		std::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> joint_one_start(ChVector<>(0, 2, 0));

	joint1->Initialize(gripperBase,        // the 1st body to connect
		leftInnerKnuckle,           // the 2nd body to connect
		false,               // the two following frames are in absolute, not relative, coords.
		joint_one_start,   // the link reference attached to 1st body
		joint_one_start);  // the link reference attached to 2nd body

	mphysicalSystem.Add(joint1);
	leftInnerKnuckle->SetPos_dt(ChVector<>(0.01, 0, 0));

    // Floor Color for Irrichlet Simulation
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.1f, 0.1f, 0.1f));
    floorBody->AddAsset(color);

    // Optionally, attach a texture to the pendulum, for better visualization
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));  // texture in ../data
    //pendulumBody->AddAsset(texture);


	// Gripper Colors
	auto gripper_gray = std::make_shared<ChColorAsset>();
	color->SetColor(ChColor(0.8f, 0.8f, 0.8f));
	gripperBase->AddAsset(texture);
	leftInnerKnuckle->AddAsset(texture);

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
